mod episode;

use chrono::{DateTime, Utc};
use dora_node_api::{
    self, DoraNode, Event, Metadata, Parameter,
    arrow::{
        array::{
            Array, BooleanArray, Int64Array, ListArray, StringArray, TimestampNanosecondArray,
            UInt64Array, make_array,
        },
        buffer::{OffsetBuffer, ScalarBuffer},
        datatypes::{DataType, Field, Schema},
        record_batch::RecordBatch,
    },
};
use dora_tracing::telemetry::deserialize_to_hashmap;
use eyre::{Context, ContextCompat};
use parquet::{arrow::AsyncArrowWriter, basic::BrotliLevel, file::properties::WriterProperties};
use serde_json;
use sha2::{Digest, Sha256};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};
use tokio::fs as tfs;
use tokio::sync::mpsc;

use crate::episode::EpisodeMetadata;

type WriterMessage = (Arc<dyn Array>, Metadata, i64);
type Writers = HashMap<
    String,
    (
        mpsc::Sender<WriterMessage>,
        tokio::task::JoinHandle<eyre::Result<()>>,
    ),
>;

struct RecordingGate {
    enabled: bool,
    episode_index: Option<i64>,
    recording: bool,
    awaiting_result: bool,
}

impl RecordingGate {
    fn new(enabled: bool) -> Self {
        Self {
            enabled,
            episode_index: None,
            recording: false,
            awaiting_result: false,
        }
    }

    fn start_episode(&mut self, episode_index: i64) -> Option<i64> {
        self.episode_index = Some(episode_index);
        self.recording = self.enabled;
        self.awaiting_result = false;
        self.recording.then_some(episode_index)
    }

    fn end_episode(&mut self) -> Option<i64> {
        let episode_index = self.recording.then_some(self.episode_index).flatten();
        self.recording = false;
        self.awaiting_result = episode_index.is_some();
        episode_index
    }

    fn finish_pending(&mut self) -> Option<i64> {
        let episode_index = self
            .awaiting_result
            .then(|| self.episode_index.take())
            .flatten();
        self.awaiting_result = false;
        episode_index
    }

    fn finish_any(&mut self) -> Option<i64> {
        let episode_index = (self.recording || self.awaiting_result)
            .then(|| self.episode_index.take())
            .flatten();
        self.recording = false;
        self.awaiting_result = false;
        episode_index
    }

    fn discard_any(&mut self) -> Option<i64> {
        let episode_index = (self.recording || self.awaiting_result)
            .then(|| self.episode_index.take())
            .flatten();
        self.recording = false;
        self.awaiting_result = false;
        episode_index
    }
}

fn capture_directory_name(
    model_backend: &str,
    model_checkpoint: &str,
    now: DateTime<Utc>,
) -> String {
    let checkpoint = Path::new(model_checkpoint)
        .file_name()
        .and_then(|name| name.to_str())
        .filter(|name| !name.is_empty());
    let model = checkpoint
        .map(|checkpoint| format!("{model_backend}_{checkpoint}"))
        .unwrap_or_else(|| model_backend.to_string());
    let model: String = model
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || matches!(character, '-' | '_') {
                character
            } else {
                '_'
            }
        })
        .collect();
    format!("{model}_{}", now.format("%Y%m%d_%H%M%S_%3f"))
}

fn task_directory_name(task_instruction: &str) -> String {
    let normalized = task_instruction.trim().to_ascii_lowercase();
    if normalized.contains("paper ball") && normalized.contains("bin") {
        return "paper_ball_to_trash_bin".to_string();
    }
    if normalized.contains("pen") && normalized.contains("pen holder") {
        return "pen_to_pen_holder".to_string();
    }
    if normalized.contains("mouse") && normalized.contains("mouse pad") {
        return "mouse_to_mouse_pad".to_string();
    }
    if normalized.contains("drink container") && normalized.contains("coaster") {
        return "drink_container_to_coaster".to_string();
    }
    if normalized.contains("cup") && normalized.contains("cup holder") {
        return "cup_to_cup_holder".to_string();
    }
    if normalized.contains("laptop") && normalized.contains("close") {
        return "close_laptop".to_string();
    }

    let mut slug = String::new();
    let mut pending_separator = false;
    for character in normalized.chars() {
        if character.is_ascii_alphanumeric() {
            if pending_separator && !slug.is_empty() {
                slug.push('_');
            }
            slug.push(character);
            pending_separator = false;
        } else {
            pending_separator = true;
        }
        if slug.len() >= 80 {
            break;
        }
    }
    let slug = slug.trim_end_matches('_');
    if slug.is_empty() {
        "unknown_task".to_string()
    } else {
        slug.to_string()
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn canonical_json(value: &serde_json::Value) -> eyre::Result<String> {
    match value {
        serde_json::Value::Array(items) => {
            let values = items
                .iter()
                .map(canonical_json)
                .collect::<eyre::Result<Vec<_>>>()?;
            Ok(format!("[{}]", values.join(",")))
        }
        serde_json::Value::Object(items) => {
            let mut keys = items.keys().collect::<Vec<_>>();
            keys.sort_unstable();
            let fields = keys
                .into_iter()
                .map(|key| {
                    Ok(format!(
                        "{}:{}",
                        serde_json::to_string(key)?,
                        canonical_json(&items[key])?
                    ))
                })
                .collect::<eyre::Result<Vec<_>>>()?;
            Ok(format!("{{{}}}", fields.join(",")))
        }
        _ => serde_json::to_string(value).map_err(eyre::Report::from),
    }
}

fn robot_contract_hashes(path: &Path) -> eyre::Result<(String, String)> {
    let bytes = std::fs::read(path).context(format!("could not read robot contract {path:?}"))?;
    let file_sha256 = sha256_hex(&bytes);
    let value: serde_json::Value =
        serde_json::from_slice(&bytes).context("robot contract must be valid JSON")?;
    let canonical = canonical_json(&value).context("could not canonicalize robot contract")?;
    Ok((sha256_hex(canonical.as_bytes()), file_sha256))
}

async fn flush_writers(writers: &mut Writers) -> eyre::Result<()> {
    for (id, (tx, join_handle)) in writers.drain() {
        drop(tx);
        join_handle
            .await
            .context("Writer thread failed")?
            .context(format!(
                "Could not close the Parquet writer for {id} parquet writer"
            ))?;
    }
    Ok(())
}

async fn finish_episode(
    episode_dir: &Path,
    writers: &mut Writers,
    metadata: &mut EpisodeMetadata,
    episode_index: i64,
    result: &str,
) -> eyre::Result<()> {
    let observed_streams = writers.keys().cloned().collect::<Vec<_>>();
    metadata.finalize_causal_evidence(&observed_streams);
    flush_writers(writers)
        .await
        .context("Failed to flush writers")?;
    metadata.finish(result);
    let metadata_file = episode_dir.join("metadata.json");
    let metadata_partial = episode_dir.join("metadata.json.partial");
    tfs::write(
        &metadata_partial,
        serde_json::to_string_pretty(metadata).context("Could not serialize metadata to JSON")?,
    )
    .await
    .context("Could not write metadata to file")?;
    tfs::rename(&metadata_partial, &metadata_file)
        .await
        .context("Could not atomically publish metadata file")?;
    println!("💾 Episode {episode_index} ended: {result}");
    Ok(())
}

async fn discard_episode(
    episode_dir: &Path,
    writers: &mut Writers,
    episode_index: i64,
) -> eyre::Result<()> {
    flush_writers(writers)
        .await
        .context("Failed to close writers for discarded episode")?;
    tfs::remove_dir_all(episode_dir).await.context(format!(
        "Could not remove discarded episode directory {episode_dir:?}"
    ))?;
    println!("Episode {episode_index} discarded by operator");
    Ok(())
}

fn first_string(data: &dyn Array) -> Option<&str> {
    let strings = data.as_any().downcast_ref::<StringArray>()?;
    (!strings.is_empty() && !strings.is_null(0)).then(|| strings.value(0))
}

fn first_bool(data: &dyn Array) -> Option<bool> {
    let values = data.as_any().downcast_ref::<BooleanArray>()?;
    (!values.is_empty() && !values.is_null(0)).then(|| values.value(0))
}

fn parameter_integer(metadata: &Metadata, name: &str) -> Option<i64> {
    match metadata.parameters.get(name) {
        Some(Parameter::Integer(value)) => Some(*value),
        _ => None,
    }
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let (node, mut events) = DoraNode::init_from_env()?;
    let dataflow_id = node.dataflow_id();
    let mut writers = Writers::new();
    let output_dir = std::env::var("OUTPUT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("out"));
    let recording_enabled = match std::env::var("ENABLE_DAGGER_RECORDING")
        .unwrap_or_else(|_| "0".to_string())
        .as_str()
    {
        "0" => false,
        "1" => true,
        value => eyre::bail!("ENABLE_DAGGER_RECORDING must be 0 or 1, got {value}"),
    };
    let evaluation_run_id = std::env::var("XPOLICY_EVAL_RUN_ID").unwrap_or_default();
    if !evaluation_run_id.trim().is_empty() && !recording_enabled {
        eyre::bail!("an evaluation run requires ENABLE_DAGGER_RECORDING=1");
    }
    let collection_mode =
        std::env::var("RECAP_COLLECTION_MODE").unwrap_or_else(|_| "MIXED".to_string());
    if !matches!(collection_mode.as_str(), "DATA_COLLECTION" | "MIXED") {
        eyre::bail!(
            "new recordings require RECAP_COLLECTION_MODE=DATA_COLLECTION or MIXED, got {collection_mode}"
        );
    }
    let model_backend = std::env::var("MODEL_BACKEND").unwrap_or_else(|_| "jiaxin".to_string());
    let model_checkpoint = std::env::var("MODEL_CHECKPOINT").unwrap_or_default();
    let task_folders_enabled = match std::env::var("DAGGER_TASK_FOLDERS")
        .unwrap_or_else(|_| "0".to_string())
        .as_str()
    {
        "0" => false,
        "1" => true,
        value => eyre::bail!("DAGGER_TASK_FOLDERS must be 0 or 1, got {value}"),
    };
    let mut current_task_instruction = std::env::var("TASK_INSTRUCTION").unwrap_or_default();
    let schema_path = std::env::var("DAGGER_SCHEMA_PATH").ok().map(PathBuf::from);
    let camera_width = std::env::var("CAMERA_TOP_WIDTH")
        .unwrap_or_else(|_| "640".to_string())
        .parse::<i64>()
        .context("CAMERA_TOP_WIDTH must be an integer")?;
    let camera_height = std::env::var("CAMERA_TOP_HEIGHT")
        .unwrap_or_else(|_| "400".to_string())
        .parse::<i64>()
        .context("CAMERA_TOP_HEIGHT must be an integer")?;
    let camera_encoding =
        std::env::var("CAMERA_TOP_ENCODING").unwrap_or_else(|_| "bgr8".to_string());
    let mut gate = RecordingGate::new(recording_enabled);
    let mut episode_path: Option<PathBuf> = None;
    let mut frame_indices: HashMap<String, i64> = HashMap::new();
    let mut episode_result_observed = false;

    let mut episode_metadata = EpisodeMetadata::new(-1);
    episode_metadata.set_session_id(dataflow_id.to_string());

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                data,
                mut metadata,
            } => {
                let mut auto_finalize_after_write = false;

                // process special events first.
                match id.as_ref() {
                    "task_instruction" => {
                        if let Some(task_instruction) = first_string(data.as_ref()) {
                            current_task_instruction = task_instruction.trim().to_string();
                            println!(
                                "DAgger recorder selected task folder: {}",
                                task_directory_name(&current_task_instruction)
                            );
                        }
                        continue;
                    }
                    "episode_index" => {
                        let episode = i64::try_from(&data).context("unexpected data type")?;

                        if episode < 0 {
                            if let Some(episode_index) = gate.end_episode() {
                                if episode_result_observed {
                                    if let Some(episode_index) = gate.finish_pending() {
                                        finish_episode(
                                            episode_path
                                                .as_deref()
                                                .context("missing episode path")?,
                                            &mut writers,
                                            &mut episode_metadata,
                                            episode_index,
                                            "saved_auto_retained",
                                        )
                                        .await?;
                                    }
                                    episode_path = None;
                                } else {
                                    println!(
                                        "⏳ Episode {episode_index} ended; awaiting result before auto-retain"
                                    );
                                }
                            }
                            continue;
                        }

                        if gate.recording || gate.awaiting_result {
                            if let Some(previous_episode) = gate.finish_any() {
                                finish_episode(
                                    episode_path.as_deref().context("missing episode path")?,
                                    &mut writers,
                                    &mut episode_metadata,
                                    previous_episode,
                                    "saved_auto_retained_before_next_episode",
                                )
                                .await?;
                            }
                            episode_path = None;
                        }
                        if let Some(episode_index) = gate.start_episode(episode) {
                            let now = Utc::now();
                            let task_directory = task_directory_name(&current_task_instruction);
                            let capture_directory =
                                capture_directory_name(&model_backend, &model_checkpoint, now);
                            let session_path = if task_folders_enabled {
                                output_dir.join(&task_directory).join(capture_directory)
                            } else {
                                output_dir.join(capture_directory)
                            };
                            let path = session_path.join(format!("episode_{episode_index:09}"));
                            if path.exists() {
                                eyre::bail!(
                                    "Refusing to overwrite existing episode directory: {path:?}"
                                );
                            }
                            std::fs::create_dir_all(&path)
                                .context(format!("could not create episode_dir for {path:?}"))?;
                            let robot_contract_hashes = if let Some(schema_path) =
                                schema_path.as_deref()
                            {
                                std::fs::copy(schema_path, path.join("schema.json")).context(
                                    format!("could not copy DAgger schema from {schema_path:?}"),
                                )?;
                                Some(robot_contract_hashes(schema_path)?)
                            } else {
                                None
                            };
                            episode_metadata = EpisodeMetadata::new(episode_index);
                            episode_metadata.set_task_instruction(current_task_instruction.clone());
                            episode_result_observed = false;
                            episode_metadata.set_session_id(dataflow_id.to_string());
                            if let Some((contract_sha256, contract_file_sha256)) =
                                robot_contract_hashes
                            {
                                episode_metadata.set_robot_contract_hashes(
                                    contract_sha256,
                                    contract_file_sha256,
                                );
                            }
                            if let Some(Parameter::String(task_id)) =
                                metadata.parameters.get("task_id")
                            {
                                episode_metadata.set_task_id(task_id.into());
                            } else if task_folders_enabled {
                                episode_metadata.set_task_id(task_directory);
                            }
                            episode_metadata.set_start_time(now);
                            frame_indices.clear();
                            println!("▶️ Start recording episode {episode_index}: {path:?}");
                            episode_path = Some(path);
                        }
                        continue;
                    }
                    "episode_failed" => {
                        if episode_metadata.task_outcome == "UNKNOWN" {
                            episode_metadata.set_task_result(
                                "ABORTED",
                                true,
                                "operator_abort",
                                None,
                            );
                        }
                        if let Some(episode_index) = gate.end_episode() {
                            println!(
                                "⏳ Aborted episode {episode_index}; awaiting result before auto-retain"
                            );
                        }
                        continue;
                    }
                    "episode_discard" => {
                        if collection_mode != "DATA_COLLECTION" {
                            eyre::bail!("episode_discard is only valid in DATA_COLLECTION mode");
                        }
                        if let Some(episode_index) = gate.discard_any() {
                            discard_episode(
                                episode_path
                                    .as_deref()
                                    .context("missing discarded episode path")?,
                                &mut writers,
                                episode_index,
                            )
                            .await?;
                            episode_path = None;
                            episode_result_observed = false;
                        }
                        continue;
                    }
                    "episode_result" => {
                        if let Some(payload) = first_string(data.as_ref()) {
                            let result: serde_json::Value = serde_json::from_str(payload)
                                .context("episode_result must be valid JSON")?;
                            episode_metadata.set_task_result(
                                result["task_outcome"].as_str().unwrap_or("UNKNOWN"),
                                result["outcome_confirmed_by_operator"]
                                    .as_bool()
                                    .unwrap_or(false),
                                result["termination_reason"].as_str().unwrap_or("unknown"),
                                result["failure_reason"].as_str().map(str::to_string),
                            );
                            episode_result_observed = true;
                            auto_finalize_after_write = gate.awaiting_result;
                        }
                    }
                    "control.event" => {
                        if let Some(event) = first_string(data.as_ref()) {
                            episode_metadata.observe_control_event(event);
                        }
                    }
                    "control.selected_action_source" => {
                        if let Some(source) = first_string(data.as_ref()) {
                            episode_metadata.observe_selected_action_source(source);
                        }
                    }
                    "label.valid" => {
                        if let Some(valid) = first_bool(data.as_ref()) {
                            episode_metadata.observe_label_valid(valid);
                        }
                    }
                    "tick" => continue,
                    _ => {}
                };

                if !gate.recording && !gate.awaiting_result {
                    continue;
                }

                if id.starts_with("observation.images.") {
                    let frame_index = frame_indices.entry(id.to_string()).or_default();
                    let camera_name = id
                        .strip_prefix("observation.images.")
                        .unwrap_or(id.as_ref());
                    metadata.parameters.insert(
                        "camera_name".into(),
                        Parameter::String(camera_name.to_string()),
                    );
                    metadata
                        .parameters
                        .insert("frame_index".into(), Parameter::Integer(*frame_index));
                    // Camera producers may declare robot-specific geometry and
                    // encoding. Keep the GR3 values only as compatibility
                    // defaults so this recorder remains robot-independent.
                    metadata
                        .parameters
                        .entry("width".into())
                        .or_insert(Parameter::Integer(camera_width));
                    metadata
                        .parameters
                        .entry("height".into())
                        .or_insert(Parameter::Integer(camera_height));
                    metadata
                        .parameters
                        .entry("encoding".into())
                        .or_insert_with(|| Parameter::String(camera_encoding.clone()));
                    *frame_index += 1;
                }

                match writers.get(&id.to_string()) {
                    None => {
                        let field_uhlc = Field::new("timestamp_uhlc", DataType::UInt64, false);
                        let field_utc_epoch = Field::new(
                            "timestamp_utc",
                            DataType::Timestamp(
                                dora_node_api::arrow::datatypes::TimeUnit::Nanosecond,
                                None,
                            ),
                            false,
                        );
                        let field_user_utc_epoch = Field::new(
                            "user_timestamp_utc",
                            DataType::Timestamp(
                                dora_node_api::arrow::datatypes::TimeUnit::Nanosecond,
                                None,
                            ),
                            false,
                        );
                        let field_trace_id = Field::new("trace_id", DataType::Utf8, true);
                        let field_span_id = Field::new("span_id", DataType::Utf8, true);
                        let field_values =
                            Arc::new(Field::new("item", data.data_type().clone(), true));
                        let field_data = Field::new(id.clone(), DataType::List(field_values), true);
                        let field_parameters = Field::new("parameters", DataType::Utf8, true);
                        let field_source_timestamp_ns =
                            Field::new("source_timestamp_ns", DataType::Int64, false);
                        let field_target_execution_timestamp_ns =
                            Field::new("target_execution_timestamp_ns", DataType::Int64, false);
                        let field_logged_timestamp_ns =
                            Field::new("logged_timestamp_ns", DataType::Int64, false);
                        let schema = Arc::new(Schema::new(vec![
                            field_trace_id,
                            field_span_id,
                            field_uhlc,
                            field_utc_epoch,
                            field_user_utc_epoch,
                            field_source_timestamp_ns,
                            field_target_execution_timestamp_ns,
                            field_logged_timestamp_ns,
                            field_parameters,
                            field_data,
                        ]));
                        // let dataflow_dir = PathBuf::from("out").join(dataflow_id.to_string());
                        // if !dataflow_dir.exists() {
                        //     std::fs::create_dir_all(&dataflow_dir)
                        //         .context("could not create dataflow_dir")?;
                        // }
                        let episode_dir = episode_path.as_ref().context("missing episode path")?;
                        if !episode_dir.exists() {
                            std::fs::create_dir_all(&episode_dir)
                                .context("could not create episode_dir")?;
                        }
                        let file =
                            tokio::fs::File::create(episode_dir.join(format!("{id}.parquet")))
                                .await
                                .context("Could not create write file")?;
                        let mut writer = AsyncArrowWriter::try_new(
                            file,
                            schema.clone(),
                            Some(
                                WriterProperties::builder()
                                    .set_compression(parquet::basic::Compression::BROTLI(
                                        BrotliLevel::default(),
                                    ))
                                    .build(),
                            ),
                        )
                        .context("Could not create parquet writer")?;
                        let (tx, mut rx) = mpsc::channel(10);

                        // Per Input thread
                        let join_handle = tokio::spawn(async move {
                            while let Some((data, metadata, logged_timestamp_ns)) = rx.recv().await
                            {
                                write_event(
                                    &mut writer,
                                    data,
                                    &metadata,
                                    logged_timestamp_ns,
                                    schema.clone(),
                                )
                                .await?;
                            }
                            writer.close().await.map(|_| ()).map_err(eyre::Report::from)
                        });
                        tx.send((
                            data.into(),
                            metadata,
                            Utc::now().timestamp_nanos_opt().unwrap_or(0),
                        ))
                        .await
                        .context("Could not send event data into writer loop")?;
                        writers.insert(id.to_string(), (tx, join_handle));
                    }
                    Some((tx, _)) => {
                        tx.send((
                            data.into(),
                            metadata,
                            Utc::now().timestamp_nanos_opt().unwrap_or(0),
                        ))
                        .await
                        .context("Could not send event data into writer loop")?;
                    }
                };

                if auto_finalize_after_write {
                    if let Some(episode_index) = gate.finish_pending() {
                        finish_episode(
                            episode_path.as_deref().context("missing episode path")?,
                            &mut writers,
                            &mut episode_metadata,
                            episode_index,
                            "saved_auto_retained",
                        )
                        .await?;
                    }
                    episode_path = None;
                }
            }
            Event::InputClosed { .. } => {}
            Event::Error(err) => {
                println!("Error: {err}");
            }
            event => {
                println!("Event: {event:#?}")
            }
        }
    }

    if let Some(episode_index) = gate.finish_any() {
        finish_episode(
            episode_path.as_deref().context("missing episode path")?,
            &mut writers,
            &mut episode_metadata,
            episode_index,
            "saved_auto_retained_on_shutdown",
        )
        .await?;
    } else {
        flush_writers(&mut writers).await?;
    }

    Ok(())
}

/// Write a row of data into the writer
async fn write_event(
    writer: &mut AsyncArrowWriter<tokio::fs::File>,
    data: Arc<dyn Array>,
    metadata: &Metadata,
    logged_timestamp_ns: i64,
    schema: Arc<Schema>,
) -> eyre::Result<()> {
    let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, data.len() as i32]));
    let field = Arc::new(Field::new("item", data.data_type().clone(), true));
    let list = ListArray::new(field, offsets, data.clone(), None);

    let timestamp = metadata.timestamp();
    let timestamp_uhlc = UInt64Array::from(vec![timestamp.get_time().0]);
    let timestamp_uhlc = make_array(timestamp_uhlc.into());
    let system_time = timestamp.get_time().to_system_time();

    let dt: DateTime<Utc> = system_time.into();
    let timestamp_utc = TimestampNanosecondArray::from(vec![dt.timestamp_nanos_opt().unwrap_or(0)]);
    let timestamp_utc = make_array(timestamp_utc.into());
    let default_timestamp_ns = dt.timestamp_nanos_opt().unwrap_or(0);
    let source_timestamp_ns = parameter_integer(metadata, "source_timestamp_ns")
        .or_else(|| parameter_integer(metadata, "timestamp_ns"))
        .unwrap_or(default_timestamp_ns);
    let target_execution_timestamp_ns =
        parameter_integer(metadata, "target_execution_timestamp_ns").unwrap_or(source_timestamp_ns);
    let source_timestamp_ns = make_array(Int64Array::from(vec![source_timestamp_ns]).into());
    let target_execution_timestamp_ns =
        make_array(Int64Array::from(vec![target_execution_timestamp_ns]).into());
    let logged_timestamp_ns = make_array(Int64Array::from(vec![logged_timestamp_ns]).into());

    let string_otel_context = metadata.open_telemetry_context();
    let otel_context = deserialize_to_hashmap(&string_otel_context);
    let traceparent = otel_context.get("traceparent");
    let trace_id = match traceparent {
        None => "",
        Some(trace) => trace.split('-').nth(1).context("Trace is malformatted")?,
    };
    let span_id = match traceparent {
        None => "",
        Some(trace) => trace.split('-').nth(2).context("Trace is malformatted")?,
    };
    let trace_id_array = StringArray::from(vec![trace_id]);
    let trace_id_array = make_array(trace_id_array.into());
    let span_id_array = StringArray::from(vec![span_id]);
    let span_id_array = make_array(span_id_array.into());

    let mut parameters = metadata.parameters.clone();

    let user_timestamp =
        if let Some(Parameter::Integer(timestamp_ns)) = metadata.parameters.get("timestamp_ns") {
            // let timestamp_ns = timestamp_ns.parse::<i64>().context("Could not parse timestamp_ns")?;
            let timestamp_ns = DateTime::<Utc>::from_timestamp_nanos(timestamp_ns.clone());
            // remove timestamp_ns from parameters
            parameters.remove("timestamp_ns");
            TimestampNanosecondArray::from(vec![timestamp_ns.timestamp_nanos_opt().unwrap_or(0)])
        } else {
            TimestampNanosecondArray::from(vec![0])
        };
    let user_timestamp = make_array(user_timestamp.into());

    let parameter_json =
        serde_json::to_string(&parameters).context("Could not serialize parameters to JSON")?;
    let parameters_array = StringArray::from(vec![parameter_json]);
    let parameters_array = make_array(parameters_array.into());

    let record = RecordBatch::try_new(
        schema,
        vec![
            trace_id_array,
            span_id_array,
            timestamp_uhlc,
            timestamp_utc,
            user_timestamp,
            source_timestamp_ns,
            target_execution_timestamp_ns,
            logged_timestamp_ns,
            parameters_array,
            make_array(list.into()),
        ],
    )
    .context("Could not create record batch with the given data")?;
    writer
        .write(&record)
        .await
        .context("Could not write recordbatch to file")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{
        EpisodeMetadata, RecordingGate, capture_directory_name, discard_episode, finish_episode,
        robot_contract_hashes, task_directory_name,
    };
    use chrono::{TimeZone, Timelike, Utc};
    use std::collections::HashMap;
    use std::path::Path;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn episode_records_immediately_and_auto_retains_after_result() {
        let mut disabled = RecordingGate::new(false);
        assert_eq!(disabled.start_episode(3), None);
        assert!(!disabled.recording);

        let mut enabled = RecordingGate::new(true);
        assert_eq!(enabled.start_episode(3), Some(3));
        assert!(enabled.recording);
        assert_eq!(enabled.end_episode(), Some(3));
        assert!(enabled.awaiting_result);
        assert_eq!(enabled.finish_pending(), Some(3));
        assert_eq!(enabled.finish_pending(), None);
    }

    #[test]
    fn active_episode_is_auto_retained_on_shutdown() {
        let mut enabled = RecordingGate::new(true);
        assert_eq!(enabled.start_episode(7), Some(7));
        assert_eq!(enabled.finish_any(), Some(7));
        assert_eq!(enabled.finish_any(), None);
    }

    #[test]
    fn discarded_episode_clears_recording_without_becoming_pending() {
        let mut enabled = RecordingGate::new(true);
        assert_eq!(enabled.start_episode(8), Some(8));
        assert_eq!(enabled.discard_any(), Some(8));
        assert!(!enabled.recording);
        assert!(!enabled.awaiting_result);
        assert_eq!(enabled.finish_any(), None);
    }

    #[test]
    fn capture_directory_contains_model_and_episode_start_time() {
        let timestamp = Utc
            .with_ymd_and_hms(2026, 7, 20, 20, 30, 45)
            .unwrap()
            .with_nanosecond(123_000_000)
            .unwrap();
        assert_eq!(
            capture_directory_name("jiaxin", "/models/one-task-gr00t", timestamp),
            "jiaxin_one-task-gr00t_20260720_203045_123"
        );
    }

    #[test]
    fn task_directory_uses_stable_desktop_cleanup_names_and_safe_fallback() {
        assert_eq!(
            task_directory_name("Pick up one paper ball and place it in the trash bin."),
            "paper_ball_to_trash_bin"
        );
        assert_eq!(
            task_directory_name("Pick up one pen and place it in the pen holder."),
            "pen_to_pen_holder"
        );
        assert_eq!(task_directory_name("Close the laptop."), "close_laptop");
        assert_eq!(
            task_directory_name("Sort red blocks / then stack them!"),
            "sort_red_blocks_then_stack_them"
        );
        assert_eq!(task_directory_name("  "), "unknown_task");
    }

    #[test]
    fn robot_contract_hash_matches_python_canonical_json() {
        let schema = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../schemas/gr3_dagger_v2.json");
        let (canonical_sha256, _) = robot_contract_hashes(&schema).unwrap();
        assert_eq!(
            canonical_sha256,
            "2c87130f6725e50de6e268c4fb2a5f1f21959bc78e93e1a0335eda4daadc3fb1"
        );
    }

    #[test]
    fn finalized_metadata_is_published_without_exposing_a_partial_file() {
        let episode_dir = std::env::temp_dir().join(format!(
            "xpolicylab-recorder-finalize-{}",
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&episode_dir).unwrap();
        let mut writers = HashMap::new();
        let mut metadata = EpisodeMetadata::new(1);

        tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(finish_episode(
                &episode_dir,
                &mut writers,
                &mut metadata,
                1,
                "saved_auto_retained",
            ))
            .unwrap();

        assert!(episode_dir.join("metadata.json").is_file());
        assert!(!episode_dir.join("metadata.json.partial").exists());
        let saved: serde_json::Value =
            serde_json::from_slice(&std::fs::read(episode_dir.join("metadata.json")).unwrap())
                .unwrap();
        assert_eq!(saved["recording_saved_successfully"], true);
        std::fs::remove_dir_all(episode_dir).unwrap();
    }

    #[test]
    fn discarded_episode_directory_is_removed_without_metadata() {
        let episode_dir = std::env::temp_dir().join(format!(
            "xpolicylab-recorder-discard-{}",
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&episode_dir).unwrap();
        std::fs::write(episode_dir.join("partial.parquet"), b"partial").unwrap();
        let mut writers = HashMap::new();

        tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(discard_episode(&episode_dir, &mut writers, 2))
            .unwrap();

        assert!(!episode_dir.exists());
    }
}
