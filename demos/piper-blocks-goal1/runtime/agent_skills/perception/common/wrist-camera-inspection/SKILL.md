---
name: wrist-camera-inspection
description: Obtain a clearer close-up of small or occluded task details, or a second viewpoint, by positioning the wrist camera with an empty gripper when the {{ robot.overview }} is insufficient. Use object-localization on the new observation.
---

# Wrist-camera inspection

Use the arm as a movable inspection camera. Bringing the wrist camera closer and
adjusting the viewing angle can reveal details that are unclear in
`{{ robot.overview_camera }}`. This is physical camera repositioning, not digital zoom.

- Do this with an empty gripper. Do not release a held object just to obtain a view.
- Use the exterior cameras to choose a clear approach and keep the gripper well clear
  of the table and objects; move in small steps and stop if clearance is uncertain.
- Inspect the fresh `wrist` image after each move. Adjust distance and angle until the
  relevant detail is clear, avoiding gripper occlusion.
{% if camera_geometry %}
- Use [object-localization](../object-localization/SKILL.md) on the new observation.
  Do not reuse old wrist-image pixels, rays or calibration after moving the camera.
{% else %}
- Use [object-localization](../object-localization/SKILL.md) on the new observation;
  do not reuse old wrist images after moving the camera.
{% endif %}

Move only when the clearer view resolves a task-relevant uncertainty, then return to a
clear pose if the arm obstructs the next manipulation.
