from . import STUDENT_MAX
"""PiPER-native tool schema, retaining upstream tool names for its event loop."""
from ..teacher_contract import response_schema


def tool_specs(mode="shadow"):
    binding = dict(request_id="", observation_sequence=0, observation_timestamp_ns="",
                   task="", proposal_sha256="", step_id=0)
    response = response_schema(binding)
    for item in response["properties"]["binding"]["properties"].values():
        item.pop("enum", None)
    response["properties"]["steps"]["anyOf"][0]["maximum"] = STUDENT_MAX
    string = {"type": "string"}

    def spec(name, description, **properties):
        return dict(type="function", name=name, description=description,
                    inputSchema=dict(type="object", additionalProperties=False,
                                     properties=properties, required=list(properties)))

    specs = [
        spec("robodojo_start", "Compatibility name: READ a fresh real PiPER observation. "
             "No simulator, reset, homing, enable, or hardware command. Shadow only.",
             task=string, output_dir=string),
        spec("pi05_infer", "Compatibility name: infer the pinned FastWAM drink policy "
             "once from the latest three original RGB images and state14. This is NOT Pi0.5. "
             "Return its unchanged H32 absolute rad/metre actions and PiPER flange FK.",
             observation_path=string, output_dir=string),
        spec("robodojo_execute", "Compatibility name: execute_shadow ONLY. Validate a "
             f"fresh FastWAM prefix (1-{STUDENT_MAX}), PiPER correction (1-5), or stop under the "
             "unchanged outcome/intent gate. Record the choice; NEVER send motion. "
             "Finish this one-decision shadow after successful validation.",
             proposal_path=string, response=response, output_dir=string),
    ]
    if mode == "live":
        specs[0]["description"] = "Read a fresh real PiPER original RGB/state observation for this explicitly authorized run. No reset or homing."
        specs[2]["description"] = (f"Validate and execute a reviewed FastWAM prefix (1-{STUDENT_MAX}) or PiPER correction (1-5) "
            "through the existing formal XPolicy owner. Wait for actual ordered owner ACKs and return fresh "
            "RGB/proprio. Edit/eef IK is recomputed after each actual ACK. Stop ends this run; no success is invented.")
    return specs
