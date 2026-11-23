"""Read-only command statistics; SDK dispatch gaps are not measured arm stops."""
from collections import Counter


class MotionStats:
    def __init__(self):
        self.samples = 0
        self.writes = 0
        self.limit_rejections = 0
        self.other_rejections = 0
        self.limit_runs = 0
        self.normalizations = 0
        self.joints = Counter()
        self.generation = None
        self.last_pass = None
        self.active = None
        self.last_run = None
        self.max_dispatch_gap_ms = 0.
        self.incomplete_runs = 0
        self.last_sequence = None
        self.missing_samples = 0

    def _end(self, sample=None):
        if self.active is None:
            return
        run = dict(self.active)
        run['observed_reject_span_ms'] = (run['last_time_ns']-run['first_time_ns'])/1e6
        run['resume_delay_ms'] = None
        run['dispatch_gap_ms'] = None
        run['recovered'] = sample is not None
        if sample is not None:
            t = sample.get('dispatch_completed_time_ns', sample['time_ns'])
            run['resume_delay_ms'] = (t-run['first_time_ns'])/1e6
            if self.last_pass is not None:
                run['dispatch_gap_ms'] = (t-self.last_pass)/1e6
                self.max_dispatch_gap_ms = max(self.max_dispatch_gap_ms, run['dispatch_gap_ms'])
        else:
            self.incomplete_runs += 1
        self.last_run = run
        self.active = None

    def observe(self, sample):
        if sample.get('event') != 'model_candidate':
            return
        sequence = sample.get('sequence')
        lost = (self.last_sequence is not None and sequence is not None
                and sequence != self.last_sequence+1)
        if lost:
            self.missing_samples += max(1, sequence-self.last_sequence-1)
        self.last_sequence = sequence
        generation = sample.get('generation')
        if lost or generation != self.generation:
            self._end()
            self.last_pass = None
            self.generation = generation
        self.samples += 1
        if sample.get('hardware_write_success'):
            self.writes += 1
            self._end(sample)
            self.last_pass = sample.get('dispatch_completed_time_ns', sample['time_ns'])
        elif 'target_outside_joint_limits' in sample.get('reasons', []):
            self.limit_rejections += 1
            self.joints.update(sample.get('invalid_target_names', []))
            if self.active is None:
                self.limit_runs += 1
                self.active = dict(generation=generation, first_time_ns=sample['time_ns'],
                                   first_sequence=sequence, rejected_samples=0)
            self.active['last_time_ns'] = sample['time_ns']
            self.active['last_sequence'] = sequence
            self.active['rejected_samples'] += 1
        else:
            self.other_rejections += 1
            # A different rejection breaks attribution to a pure limit run.
            self._end()
            self.last_pass = None
        self.normalizations += bool(sample.get('boundary_adjustments'))

    def snapshot(self):
        active = dict(self.active) if self.active else None
        if active:
            active['observed_reject_span_ms'] = (active['last_time_ns']-active['first_time_ns'])/1e6
        return dict(samples=self.samples, successful_dispatches=self.writes,
                    limit_rejected_samples=self.limit_rejections,
                    other_rejected_samples=self.other_rejections,
                    limit_runs_started=self.limit_runs, incomplete_limit_runs=self.incomplete_runs,
                    boundary_normalized_samples=self.normalizations,
                    invalid_joint_counts=dict(self.joints), active_limit_run=active,
                    last_limit_run=self.last_run,
                    max_limit_dispatch_gap_ms=self.max_dispatch_gap_ms,
                    missing_sequence_samples=self.missing_samples,
                    timing_definition='successful SDK command gap; not measured physical stop duration')
