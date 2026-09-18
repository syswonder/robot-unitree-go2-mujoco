config:
  profile_file:
    type: string
    default: /scene_profile/multifloor.yaml
    description: Read-only scene-level floor and stair annotation mounted by the deployment.
  startup_floor:
    type: int
    default: 1
    description: Physical floor at simulator startup.
  bridge_url:
    type: string
    default: http://127.0.0.1:8766
    description: Local simulator bridge health and acknowledged command endpoint.
