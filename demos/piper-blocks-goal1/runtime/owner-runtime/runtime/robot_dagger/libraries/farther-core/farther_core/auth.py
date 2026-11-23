from livekit import api


class Identity:
    ROBOT = "robot"
    TELEOPERATOR = "teleoperator"
    VR = "vr-user"


def generate_token(room_name, identity=None, name=None):
    """
    Generate a LiveKit access token for room access.

    Args:
        room_name: The name of the room to join
        identity: The participant identity
        name: The display name (default: same as identity)

    Returns:
        JWT token string
    """
    if not identity:
        identity = Identity.ROBOT

    if not name:
        name = identity

    # Create token with video grants
    token = (
        api.AccessToken()
        .with_identity(identity)
        .with_name(name)
        .with_grants(
            api.VideoGrants(
                room_join=True,
                room=room_name,
            )
        )
        .to_jwt()
    )

    return token
