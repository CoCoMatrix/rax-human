#!/usr/bin/env python3
"""Inspect or atomically fetch an immutable model bundle from Aliyun OSS."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import time

import oss2
import requests


def credentials() -> tuple[str, str]:
    key_id = os.getenv("ALIBABA_CLOUD_ACCESS_KEY_ID")
    key_secret = os.getenv("ALIBABA_CLOUD_ACCESS_KEY_SECRET")
    if not key_id or not key_secret:
        raise SystemExit(
            "ALIBABA_CLOUD_ACCESS_KEY_ID and ALIBABA_CLOUD_ACCESS_KEY_SECRET are required"
        )
    return key_id, key_secret


def parse_metadata(values: list[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise SystemExit(f"invalid --metadata value: {value!r}")
        key, item = value.split("=", 1)
        if not key or key in result:
            raise SystemExit(f"invalid or duplicate metadata key: {key!r}")
        result[key] = item
    return result


def relative_key(key: str, prefix: str) -> Path:
    if not key.startswith(prefix):
        raise ValueError(f"OSS key escaped requested prefix: {key!r}")
    relative = PurePosixPath(key.removeprefix(prefix))
    if not relative.parts or relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"unsafe OSS object key: {key!r}")
    return Path(*relative.parts)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def enable_resumable_part_retries(attempts: int) -> None:
    """Keep one transient range failure from cancelling every download worker.

    Aliyun's SDK records a part only after its byte count and CRC validate, so
    retrying the private per-part operation is safe: a retry seeks back to the
    same offset and overwrites the incomplete range before committing it.
    """
    downloader = getattr(oss2.resumable, "_ResumableDownloader", None)
    method_name = "_ResumableDownloader__download_part"
    original = getattr(downloader, method_name, None)
    if downloader is None or original is None:
        raise RuntimeError("installed oss2 SDK does not expose resumable part downloads")

    def download_part_with_retry(self: object, part: object) -> None:
        for attempt in range(1, attempts + 1):
            try:
                original(self, part)
                return
            except (
                oss2.exceptions.RequestError,
                requests.exceptions.RequestException,
            ):
                if attempt == attempts:
                    raise
                time.sleep(min(2.0, 0.1 * (2 ** (attempt - 1))))

    setattr(downloader, method_name, download_part_with_retry)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bucket", required=True)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--destination", type=Path)
    parser.add_argument("--expected-bytes", type=int)
    parser.add_argument("--metadata", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--request-timeout-seconds", type=int, default=120)
    parser.add_argument("--attempts", type=int, default=5)
    args = parser.parse_args()
    if args.threads < 1 or args.request_timeout_seconds < 1 or args.attempts < 1:
        raise SystemExit("--threads, --request-timeout-seconds and --attempts must be positive")
    if args.inspect and args.destination is not None:
        raise SystemExit("--inspect does not accept --destination")
    if not args.inspect and (args.destination is None or args.expected_bytes is None):
        raise SystemExit("fetch requires --destination and --expected-bytes")

    prefix = args.prefix.strip("/") + "/"
    key_id, key_secret = credentials()
    bucket = oss2.Bucket(
        oss2.Auth(key_id, key_secret),
        args.endpoint,
        args.bucket,
        session=oss2.Session(pool_size=args.threads),
        connect_timeout=args.request_timeout_seconds,
    )
    objects = [
        item
        for item in oss2.ObjectIterator(bucket, prefix=prefix)
        if not item.key.endswith("/")
    ]
    if not objects:
        raise SystemExit(f"no objects found under oss://{args.bucket}/{prefix}")
    total_bytes = sum(item.size for item in objects)
    if args.inspect:
        print(
            json.dumps(
                {
                    "uri": f"oss://{args.bucket}/{prefix}",
                    "endpoint": args.endpoint,
                    "object_count": len(objects),
                    "total_bytes": total_bytes,
                    "sample": [
                        {
                            "path": relative_key(item.key, prefix).as_posix(),
                            "bytes": item.size,
                            "etag": item.etag,
                        }
                        for item in objects[:50]
                    ],
                },
                indent=2,
                sort_keys=True,
            )
        )
        return
    enable_resumable_part_retries(min(args.attempts, 6))
    if total_bytes != args.expected_bytes:
        raise SystemExit(
            f"OSS size changed: expected {args.expected_bytes}, found {total_bytes}"
        )

    destination = args.destination.expanduser().resolve()
    staging = destination.with_name(f".{destination.name}.partial")
    if destination.exists():
        raise SystemExit(f"immutable destination already exists: {destination}")
    staging.mkdir(parents=True, exist_ok=True)
    resume_store = oss2.ResumableDownloadStore(root=str(staging / ".oss-resume"))
    print(
        f"Syncing {len(objects)} objects ({total_bytes / 1024**3:.3f} GiB) "
        f"from oss://{args.bucket}/{prefix}",
        flush=True,
    )
    for index, item in enumerate(objects, 1):
        relative = relative_key(item.key, prefix)
        output = staging / relative
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.is_file() and output.stat().st_size == item.size:
            print(f"[{index}/{len(objects)}] reuse {relative}", flush=True)
            continue
        print(
            f"[{index}/{len(objects)}] download {relative} "
            f"({item.size / 1024**3:.3f} GiB)",
            flush=True,
        )
        for attempt in range(1, args.attempts + 1):
            try:
                oss2.resumable_download(
                    bucket,
                    item.key,
                    str(output),
                    multiget_threshold=16 * 1024 * 1024,
                    part_size=16 * 1024 * 1024,
                    num_threads=args.threads,
                    store=resume_store,
                )
                break
            except (oss2.exceptions.RequestError, requests.exceptions.RequestException):
                if attempt == args.attempts:
                    raise
                delay = min(30, 2 ** (attempt - 1))
                print(
                    f"[{index}/{len(objects)}] transient OSS error; "
                    f"resume attempt {attempt + 1}/{args.attempts} in {delay}s",
                    flush=True,
                )
                time.sleep(delay)
        if output.stat().st_size != item.size:
            raise RuntimeError(f"size mismatch after download: {relative}")

    shutil.rmtree(staging / ".oss-resume", ignore_errors=True)
    checksums: list[tuple[str, str]] = []
    print("Computing SHA-256 checksums...", flush=True)
    for item in objects:
        relative = relative_key(item.key, prefix)
        checksums.append((sha256(staging / relative), relative.as_posix()))
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source": {
            "provider": "aliyun-oss",
            "uri": f"oss://{args.bucket}/{prefix}",
            "endpoint": args.endpoint,
            "object_count": len(objects),
            "total_bytes": total_bytes,
        },
        "model": parse_metadata(args.metadata),
        "objects": [
            {
                "path": relative_key(item.key, prefix).as_posix(),
                "bytes": item.size,
                "etag": item.etag,
                "sha256": checksum,
            }
            for item, (checksum, _) in zip(objects, checksums, strict=True)
        ],
    }
    (staging / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (staging / "SHA256SUMS").write_text(
        "".join(f"{checksum}  {path}\n" for checksum, path in checksums),
        encoding="utf-8",
    )
    os.replace(staging, destination)
    print(f"Published immutable model bundle: {destination}", flush=True)


if __name__ == "__main__":
    main()
