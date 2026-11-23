#!/usr/bin/env python3
"""Resiliently fill a sparse OSS object into a preallocated local file.

This is an intake helper for the two large Object50 checkpoint files.  It is
deliberately resumable: allocated data extents are kept, and transient OSS
failures only defer a range to a later round.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import os
import time

import oss2
import requests


def complete(fd: int, start: int, end: int) -> bool:
    """Whether [start, end) is backed by written data (not an unwritten extent)."""
    try:
        data = os.lseek(fd, start, os.SEEK_DATA)
        if data != start:
            return False
        hole = os.lseek(fd, start, os.SEEK_HOLE)
        return hole >= end
    except OSError:
        return False


def ranges(fd: int, size: int, chunk: int) -> list[tuple[int, int]]:
    result = []
    for start in range(0, size, chunk):
        end = min(size, start + chunk)
        if not complete(fd, start, end):
            result.append((start, end))
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--access-key-id", default=os.getenv("ALIBABA_CLOUD_ACCESS_KEY_ID"))
    parser.add_argument("--access-key-secret", default=os.getenv("ALIBABA_CLOUD_ACCESS_KEY_SECRET"))
    parser.add_argument("--bucket", required=True)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--key", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source", action="append", default=[])
    parser.add_argument("--size", type=int, required=True)
    parser.add_argument("--chunk-mib", type=int, default=16)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--attempts", type=int, default=12)
    parser.add_argument("--rounds", type=int, default=0,
                        help="0 means keep retrying until complete")
    args = parser.parse_args()
    if not args.access_key_id or not args.access_key_secret:
        raise SystemExit("OSS credentials must be passed or exported as ALIBABA_CLOUD_ACCESS_KEY_ID/SECRET")
    if args.chunk_mib < 1 or args.threads < 1 or args.attempts < 1:
        raise SystemExit("chunk, threads and attempts must be positive")

    output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    out_fd = os.open(output, os.O_RDWR | os.O_CREAT)
    try:
        os.ftruncate(out_fd, args.size)
        sources: list[tuple[int, int]] = []
        for path in args.source:
            if os.path.isfile(path):
                fd = os.open(path, os.O_RDONLY)
                sources.append((fd, os.stat(path).st_size))

        # Sign once, then use bounded independent HTTP requests.  The OSS
        # SDK's streamed ``read()`` has no read deadline in this environment;
        # requests' tuple timeout prevents one stalled range from blocking a
        # whole worker forever.  Range is an ordinary request header and does
        # not alter the signed object identity.
        bucket = oss2.Bucket(
            oss2.Auth(args.access_key_id, args.access_key_secret),
            args.endpoint,
            args.bucket,
            connect_timeout=20,
        )
        signed_url = bucket.sign_url("GET", args.key, 3600)
        chunk = args.chunk_mib * 1024 * 1024
        round_no = 0
        while True:
            round_no += 1
            missing = ranges(out_fd, args.size, chunk)
            if not missing:
                os.fsync(out_fd)
                print(f"complete: {args.output} ({args.size} bytes)", flush=True)
                return

            # First use any valid extents left by a previous downloader.
            copied = 0
            still_missing: list[tuple[int, int]] = []
            for start, end in missing:
                copied_from_source = False
                for src_fd, src_size in sources:
                    if end <= src_size and complete(src_fd, start, end):
                        body = os.pread(src_fd, end - start, start)
                        if len(body) == end - start:
                            os.pwrite(out_fd, body, start)
                            copied += end - start
                            copied_from_source = True
                            break
                if not copied_from_source:
                    still_missing.append((start, end))
            missing = still_missing
            if copied:
                print(f"round {round_no}: copied {copied / 1024**3:.2f} GiB from prior partial", flush=True)
            if not missing:
                continue

            def fetch(span: tuple[int, int]) -> tuple[tuple[int, int], bytes | None, str | None]:
                start, end = span
                last = "unknown error"
                for attempt in range(1, args.attempts + 1):
                    try:
                        response = requests.get(
                            signed_url,
                            headers={"Range": f"bytes={start}-{end - 1}"},
                            timeout=(15, 90),
                        )
                        response.raise_for_status()
                        body = response.content
                        if len(body) != end - start:
                            raise RuntimeError(f"short range ({len(body)} != {end - start})")
                        return span, body, None
                    except Exception as exc:  # transient SDK/proxy/TLS errors
                        last = repr(exc)
                        time.sleep(min(8.0, 0.25 * (2 ** min(attempt - 1, 5))))
                return span, None, last

            failures = 0
            fetched = 0
            with ThreadPoolExecutor(max_workers=args.threads) as pool:
                futures = [pool.submit(fetch, span) for span in missing]
                for future in as_completed(futures):
                    span, body, error = future.result()
                    if body is None:
                        failures += 1
                        print(f"defer {span[0]}:{span[1]}: {error}", flush=True)
                        continue
                    os.pwrite(out_fd, body, span[0])
                    fetched += len(body)
                    if fetched // (1024**3) != (fetched - len(body)) // (1024**3):
                        print(f"round {round_no}: fetched {fetched / 1024**3:.2f} GiB", flush=True)

            print(
                f"round {round_no}: {len(missing) - failures} ranges fetched, "
                f"{failures} deferred, {len(ranges(out_fd, args.size, chunk))} ranges remain",
                flush=True,
            )
            if args.rounds and round_no >= args.rounds:
                raise SystemExit("retry rounds exhausted")
            time.sleep(2.0 if failures else 0.2)
    finally:
        for fd, _ in locals().get("sources", []):
            os.close(fd)
        os.close(out_fd)


if __name__ == "__main__":
    main()
