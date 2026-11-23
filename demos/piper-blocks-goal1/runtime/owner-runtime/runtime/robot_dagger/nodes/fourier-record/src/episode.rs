use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::env;

#[derive(Debug, Serialize, Deserialize)]
pub struct SourceIdentity {
    pub repo: String,
    pub commit: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RobotContractIdentity {
    pub id: String,
    pub sha256: Option<String>,
    pub file_sha256: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct BehaviorPolicyIdentity {
    pub policy_id: String,
    pub checkpoint_uri: String,
    pub checkpoint_sha256: Option<String>,
    pub training_dataset_id: Option<String>,
    pub condition: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RecapCollectionIdentity {
    pub schema_version: String,
    pub collection_batch_id: String,
    pub collection_mode: String,
    pub recap_iteration: String,
    pub parent_dataset_id: Option<String>,
    pub source: SourceIdentity,
    pub robot_contract: RobotContractIdentity,
    pub behavior_policy: BehaviorPolicyIdentity,
    pub outcome_label_schema_version: String,
    pub extensions: BTreeMap<String, serde_json::Value>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CausalEvidenceIdentity {
    pub schema_version: String,
    pub frame_key: Vec<String>,
    pub decision_key: String,
    pub observation_timestamp: String,
    pub action_timestamp: String,
    pub streams: BTreeMap<String, String>,
    pub observed_streams: Vec<String>,
    pub missing_required_streams: Vec<String>,
    pub expert_control_observed: bool,
    pub expert_evidence_required: bool,
    pub complete: bool,
}

impl CausalEvidenceIdentity {
    fn new() -> Self {
        Self {
            schema_version: "xpolicy.recap.causal_evidence.v1".to_string(),
            frame_key: vec!["episode_id".to_string(), "frame_index".to_string()],
            decision_key: "control_tick_id".to_string(),
            observation_timestamp: "source_timestamp_ns".to_string(),
            action_timestamp: "target_execution_timestamp_ns".to_string(),
            streams: BTreeMap::from([
                (
                    "observation_state".to_string(),
                    "observation.state_robot".to_string(),
                ),
                (
                    "observation_images".to_string(),
                    "observation.images.*".to_string(),
                ),
                (
                    "policy_proposal".to_string(),
                    "action.model_robot".to_string(),
                ),
                (
                    "teleop_mapped_raw".to_string(),
                    "action.expert_proposed_robot".to_string(),
                ),
                (
                    "expert_aligned_unclipped".to_string(),
                    "action.expert_aligned_unclipped_robot".to_string(),
                ),
                (
                    "expert_safe".to_string(),
                    "action.expert_safe_robot".to_string(),
                ),
                (
                    "executed_action".to_string(),
                    "action.executed_robot".to_string(),
                ),
                (
                    "selected_action_source".to_string(),
                    "control.selected_action_source".to_string(),
                ),
                ("control_event".to_string(), "control.event".to_string()),
                (
                    "takeover_event".to_string(),
                    "control.takeover_event".to_string(),
                ),
                ("expert_label_valid".to_string(), "label.valid".to_string()),
                ("episode_result".to_string(), "episode_result".to_string()),
                ("policy_info".to_string(), "model.policy_info".to_string()),
            ]),
            observed_streams: Vec::new(),
            missing_required_streams: Vec::new(),
            expert_control_observed: false,
            expert_evidence_required: false,
            complete: false,
        }
    }

    fn observe_expert_control(&mut self) {
        self.expert_control_observed = true;
    }

    fn finalize(&mut self, collection_mode: &str, observed_streams: &[String]) {
        let mut observed = observed_streams.to_vec();
        observed.sort();
        observed.dedup();
        let mut required = vec![
            "observation.state_robot",
            "action.executed_robot",
            "control.selected_action_source",
            "control.event",
            "label.valid",
            "episode_result",
        ];
        if matches!(collection_mode, "AUTONOMOUS" | "INTERVENTION" | "MIXED") {
            required.push("action.model_robot");
        }
        self.expert_evidence_required = matches!(
            collection_mode,
            "DATA_COLLECTION" | "DEMONSTRATION" | "INTERVENTION"
        ) || (collection_mode == "MIXED"
            && self.expert_control_observed);
        if self.expert_evidence_required {
            required.push("action.expert_safe_robot");
        }
        let mut missing = required
            .into_iter()
            .filter(|stream| !observed.iter().any(|item| item == stream))
            .map(str::to_string)
            .collect::<Vec<_>>();
        if !observed
            .iter()
            .any(|stream| stream.starts_with("observation.images."))
        {
            missing.push("observation.images.*".to_string());
        }
        missing.sort();
        self.complete = missing.is_empty();
        self.observed_streams = observed;
        self.missing_required_streams = missing;
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct EvaluationEpisodeIdentity {
    pub schema_version: String,
    pub run_id: String,
    pub suite_id: String,
    pub suite_revision: String,
    pub task_id: String,
    pub task_revision: String,
    pub trial_id: String,
    pub case_id: Option<String>,
    pub robot_contract_sha256: Option<String>,
    pub evidence_retention_policy: String,
}

impl EvaluationEpisodeIdentity {
    fn from_environment(episode_index: i64, task_id: Option<&str>) -> Option<Self> {
        let run_id = optional_env("XPOLICY_EVAL_RUN_ID")?;
        let case_ids = optional_env("XPOLICY_EVAL_CASE_IDS")
            .map(|value| {
                value
                    .split(',')
                    .map(str::trim)
                    .filter(|value| !value.is_empty())
                    .map(str::to_string)
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let case_id = if case_ids.is_empty() {
            optional_env("XPOLICY_EVAL_CASE_ID")
        } else {
            usize::try_from(episode_index)
                .ok()
                .map(|index| case_ids[index % case_ids.len()].clone())
        };
        Some(Self {
            schema_version: "xpolicy.eval.episode_link.v1".to_string(),
            run_id,
            suite_id: env::var("XPOLICY_EVAL_SUITE_ID").unwrap_or("adhoc".to_string()),
            suite_revision: env::var("XPOLICY_EVAL_SUITE_REVISION").unwrap_or("v1".to_string()),
            task_id: task_id.unwrap_or("unknown").to_string(),
            task_revision: env::var("XPOLICY_EVAL_TASK_REVISION").unwrap_or("v1".to_string()),
            trial_id: format!("trial-{episode_index:06}"),
            case_id,
            robot_contract_sha256: None,
            evidence_retention_policy: env::var("EVIDENCE_RETENTION_POLICY")
                .unwrap_or("AUTO_RETAIN_ALL_EVAL".to_string()),
        })
    }
}

fn optional_env(name: &str) -> Option<String> {
    env::var(name).ok().filter(|value| !value.trim().is_empty())
}

#[derive(Debug, Serialize, Deserialize)]
pub struct EpisodeMetadata {
    pub episode_index: i64,
    pub episode_id: String,
    pub rollout_id: Option<String>,
    pub intervention_count: u64,
    pub task_id: Option<String>,
    pub task_instruction: String,
    pub start_time: Option<DateTime<Utc>>,
    pub end_time: Option<DateTime<Utc>>,
    pub session_id: Option<String>,
    pub notes: String,
    pub pilot: String,
    pub operator: String,
    pub operator_id: String,
    pub robot_id: String,
    pub machine_id: String,
    pub station_id: String,
    pub equipment: String,
    pub camera_type: String,
    pub result: Option<String>,
    pub model_backend: String,
    pub model_checkpoint: String,
    pub dagger_iteration: String,
    pub retarget_mapping_version: String,
    pub controller_version: String,
    pub storage_format: String,
    pub schema_version: String,
    pub robot_type: String,
    pub profile_id: String,
    pub task_outcome: String,
    pub outcome_confirmed_by_operator: bool,
    pub termination_reason: String,
    pub failure_reason: Option<String>,
    pub recording_saved_successfully: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evaluation: Option<EvaluationEpisodeIdentity>,
    pub recap: RecapCollectionIdentity,
    pub causal_evidence: CausalEvidenceIdentity,
}

impl EpisodeMetadata {
    pub fn new(episode_index: i64) -> Self {
        let notes = env::var("DAQ_NOTES").unwrap_or("".to_string());
        let pilot = env::var("DAQ_PILOT").unwrap_or("-1".to_string());
        let operator = env::var("DAQ_OPERATOR").unwrap_or("-1".to_string());
        let robot_id = env::var("ROBOT_ID").unwrap_or("-1".to_string());
        let machine_id = env::var("DAQ_MACHINE_ID").unwrap_or("".to_string());
        let station_id = env::var("DAQ_STATION_ID").unwrap_or("-1".to_string());
        let equipment = env::var("DAQ_EQUIPMENT_TYPE").unwrap_or("".to_string());
        let camera_type = env::var("DAQ_CAMERA_TYPE").unwrap_or("".to_string());
        let task_id = env::var("DAQ_TASK_ID").ok().filter(|s| !s.is_empty());
        let evaluation =
            EvaluationEpisodeIdentity::from_environment(episode_index, task_id.as_deref());
        let model_backend = env::var("MODEL_BACKEND").unwrap_or("jiaxin".to_string());
        let model_checkpoint = env::var("MODEL_CHECKPOINT").unwrap_or_default();
        let dagger_iteration = env::var("DAGGER_ITERATION").unwrap_or("0".to_string());
        let task_instruction = env::var("TASK_INSTRUCTION").unwrap_or_default();
        let retarget_mapping_version =
            env::var("RETARGET_MAPPING_VERSION").unwrap_or("unknown".to_string());
        let controller_version = env::var("CONTROLLER_VERSION").unwrap_or("unknown".to_string());
        let storage_format =
            env::var("DAGGER_STORAGE_FORMAT").unwrap_or("xpolicylab_dagger_parquet_v1".to_string());
        let schema_version =
            env::var("DAGGER_SCHEMA_VERSION").unwrap_or("gr3_dagger_v2".to_string());
        let robot_type = env::var("ROBOT_TYPE").unwrap_or("gr3".to_string());
        let profile_id = env::var("ROBOT_PROFILE_ID").unwrap_or_default();
        let recap_iteration =
            optional_env("RECAP_ITERATION").unwrap_or_else(|| dagger_iteration.clone());
        let collection_batch_id = env::var("COLLECTION_BATCH_ID").unwrap_or_default();
        let collection_mode = env::var("RECAP_COLLECTION_MODE").unwrap_or("MIXED".to_string());
        let source_repo =
            env::var("XPOLICYLAB_SOURCE_REPO").unwrap_or("CoCoMatrix/xpolicylabdagger".to_string());
        let source_commit = env::var("XPOLICYLAB_SOURCE_COMMIT").unwrap_or("unknown".to_string());
        let behavior_condition =
            env::var("RECAP_POLICY_CONDITION").unwrap_or("UNCONDITIONED".to_string());
        let recap = RecapCollectionIdentity {
            schema_version: "xpolicy.recap.collection.v1".to_string(),
            collection_batch_id,
            collection_mode,
            recap_iteration,
            parent_dataset_id: optional_env("RECAP_PARENT_DATASET_ID"),
            source: SourceIdentity {
                repo: source_repo,
                commit: source_commit,
            },
            robot_contract: RobotContractIdentity {
                id: schema_version.clone(),
                sha256: None,
                file_sha256: None,
            },
            behavior_policy: BehaviorPolicyIdentity {
                policy_id: model_backend.clone(),
                checkpoint_uri: model_checkpoint.clone(),
                checkpoint_sha256: optional_env("MODEL_CHECKPOINT_SHA256"),
                training_dataset_id: optional_env("MODEL_TRAINING_DATASET_ID"),
                condition: behavior_condition,
            },
            outcome_label_schema_version: "xpolicy.recap.outcome.v1".to_string(),
            extensions: BTreeMap::new(),
        };

        Self {
            episode_index,
            episode_id: format!("episode_{episode_index:09}"),
            rollout_id: None,
            intervention_count: 0,
            task_id,
            task_instruction,
            start_time: None,
            end_time: None,
            session_id: None,
            notes,
            pilot,
            operator: operator.clone(),
            operator_id: operator,
            robot_id,
            machine_id,
            station_id,
            equipment,
            camera_type,
            result: None,
            model_backend,
            model_checkpoint,
            dagger_iteration,
            retarget_mapping_version,
            controller_version,
            storage_format,
            schema_version,
            robot_type,
            profile_id,
            task_outcome: "UNKNOWN".to_string(),
            outcome_confirmed_by_operator: false,
            termination_reason: "unknown".to_string(),
            failure_reason: None,
            recording_saved_successfully: false,
            evaluation,
            recap,
            causal_evidence: CausalEvidenceIdentity::new(),
        }
    }

    pub fn set_session_id(&mut self, session_id: String) {
        if self.recap.collection_batch_id.is_empty() {
            self.recap.collection_batch_id = session_id.clone();
        }
        self.rollout_id = Some(format!("{session_id}:{}", self.episode_id));
        self.session_id = Some(session_id);
    }

    pub fn set_robot_contract_hashes(&mut self, sha256: String, file_sha256: String) {
        self.recap.robot_contract.sha256 = Some(sha256.clone());
        self.recap.robot_contract.file_sha256 = Some(file_sha256);
        if let Some(evaluation) = self.evaluation.as_mut() {
            evaluation.robot_contract_sha256 = Some(sha256);
        }
    }

    pub fn set_task_id(&mut self, task_id: String) {
        self.task_id = Some(task_id.clone());
        if let Some(evaluation) = self.evaluation.as_mut() {
            evaluation.task_id = task_id;
        }
    }

    pub fn set_task_instruction(&mut self, task_instruction: String) {
        self.task_instruction = task_instruction;
    }

    pub fn set_start_time(&mut self, start_time: DateTime<Utc>) {
        self.start_time = Some(start_time);
    }

    pub fn observe_control_event(&mut self, event: &str) {
        if event == "takeover_request" {
            self.intervention_count += 1;
            self.causal_evidence.observe_expert_control();
        }
    }

    pub fn observe_selected_action_source(&mut self, source: &str) {
        if source.eq_ignore_ascii_case("EXPERT") {
            self.causal_evidence.observe_expert_control();
        }
    }

    pub fn observe_label_valid(&mut self, valid: bool) {
        if valid {
            self.causal_evidence.observe_expert_control();
        }
    }

    pub fn set_task_result(
        &mut self,
        task_outcome: &str,
        confirmed: bool,
        termination_reason: &str,
        failure_reason: Option<String>,
    ) {
        self.task_outcome = task_outcome.to_string();
        self.outcome_confirmed_by_operator = confirmed;
        self.termination_reason = termination_reason.to_string();
        self.failure_reason = failure_reason;
    }

    pub fn finalize_causal_evidence(&mut self, observed_streams: &[String]) {
        self.causal_evidence
            .finalize(&self.recap.collection_mode, observed_streams);
    }

    pub fn finish(&mut self, result: &str) {
        self.end_time = Some(Utc::now());
        self.result = Some(result.to_string());
        self.recording_saved_successfully = true;
    }
}

#[cfg(test)]
mod tests {
    use super::EpisodeMetadata;

    #[test]
    fn saved_episode_separates_recording_result_from_task_outcome() {
        let mut metadata = EpisodeMetadata::new(7);
        metadata.finalize_causal_evidence(&[
            "observation.images.camera_top".to_string(),
            "observation.state_robot".to_string(),
            "action.model_robot".to_string(),
            "action.expert_safe_robot".to_string(),
            "action.executed_robot".to_string(),
            "control.selected_action_source".to_string(),
            "control.event".to_string(),
            "label.valid".to_string(),
            "episode_result".to_string(),
        ]);
        metadata.set_task_result("ABORTED", true, "operator_abort", None);
        metadata.finish("saved");
        assert_eq!(metadata.result.as_deref(), Some("saved"));
        assert_eq!(metadata.task_outcome, "ABORTED");
        assert!(metadata.recording_saved_successfully);
        assert!(metadata.end_time.is_some());
        assert_eq!(metadata.recap.schema_version, "xpolicy.recap.collection.v1");
        assert_eq!(
            metadata.recap.outcome_label_schema_version,
            "xpolicy.recap.outcome.v1"
        );
        assert!(metadata.causal_evidence.complete);
        assert!(metadata.causal_evidence.missing_required_streams.is_empty());
    }

    #[test]
    fn causal_evidence_requirements_follow_collection_mode() {
        let common = vec![
            "observation.images.cam_head".to_string(),
            "observation.state_robot".to_string(),
            "action.executed_robot".to_string(),
            "control.selected_action_source".to_string(),
            "control.event".to_string(),
            "label.valid".to_string(),
            "episode_result".to_string(),
        ];
        let mut autonomous = EpisodeMetadata::new(1);
        autonomous.recap.collection_mode = "AUTONOMOUS".to_string();
        let mut autonomous_streams = common.clone();
        autonomous_streams.push("action.model_robot".to_string());
        autonomous.finalize_causal_evidence(&autonomous_streams);
        assert!(autonomous.causal_evidence.complete);

        let mut intervention = EpisodeMetadata::new(2);
        intervention.recap.collection_mode = "INTERVENTION".to_string();
        intervention.finalize_causal_evidence(&autonomous_streams);
        assert!(!intervention.causal_evidence.complete);
        assert_eq!(
            intervention.causal_evidence.missing_required_streams,
            vec!["action.expert_safe_robot"]
        );
    }
}
