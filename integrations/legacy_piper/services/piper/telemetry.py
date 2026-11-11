from .owner import OwnerReader


class Telemetry:
    def snapshot(self):
        try:
            data=OwnerReader().state()
            return dict(kind='piper',ok=True,mode='NO_SEND adapter',robot_state=data['robot_state'],
                        robot_status=data['robot_status'],cameras=data['cameras'])
        except Exception as exc:
            return dict(kind='piper',ok=False,error=str(exc))
