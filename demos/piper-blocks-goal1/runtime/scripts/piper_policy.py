"""Existing XPolicy policy node with a faster LOCAL reviewed-idle poll.

No SDK, queued-action blending, automatic recovery or weakened validation.
Other tasks and cloud no_action/error backoff keep the installed runtime behavior.
"""
import time


def local_review_runtime(base, *, clock=time.monotonic):
    class LocalReviewRuntime(base):
        def tick(self,*args,**kwargs):
            result=super().tick(*args,**kwargs)
            reply=result.inference
            if (result.no_action_accepted and self.runtime.allow_review_wait
                    and reply is not None and reply.error is None
                    and reply.no_action is not None
                    and reply.no_action.transport_receipt.get('mode')=='continuous_review_wait'):
                self.retry_at=min(self.retry_at,clock()+.05)
            return result
    return LocalReviewRuntime


def main():
    import os
    if os.environ.get('PIPER_CONTINUOUS_REVIEW')!='1' or os.environ.get('POLICY_URL')!='ws://127.0.0.1:19027':
        raise ValueError('Fast review polling requires this local RAX bridge')
    # The installed node imports this exact module. Patch only this process,
    # before importing the node; never edit the shared owner/vendor files.
    from XPolicyLab.real_world import inference
    inference.ActionChunkPolicyRuntime=local_review_runtime(inference.ActionChunkPolicyRuntime)
    from real_world.robots.piper_x_dual.policy_node import main as run
    run()

if __name__=='__main__':main()
