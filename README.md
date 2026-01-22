# Unitree Interface

## Modes
- **std::monostate**: A stand-in for a logically uninitialized mode. Since mode creation methods are private (the node cannot access them, only the sdk wrapper and transition structs can), we initialize to a monostate and immediately perform a transition to IdleMode.
- **IdleMode**: A mode wherein no commands will be forwarded to the G1.
- **HighLevelMode**: A mode wherein high-level actions (as defined by unitree) can be forwarded to the G1.
- **LowLevelMode**: A mode wherein we can control motor gains and provide joint-level commands.
- **EmergencyMode**: On transitioning to this mode, the G1 will immediately attempt to regain high-level control and execute joint-level damping. Transitioning away from this mode is not allowed.

Our modes don't truly reflect unitree's internal controller state. We can land up in a situation wherein we transition from LowLevelMode to IdleMode and attempt to transition to EmergencyMode. We must cognizant of the internal controller state before attempting to enter damping mode while transitioning to EmergencyMode.

Currently, we attempt to enter HighLevelMode each time we transition from either IdleMode to EmergencyMode (and similarly for LowLevelMode). It might be a better idea to enforce the invariant that IdleMode always exists with the high-level control services active to make transitioning to EmergencyMode both faster and potentially more reliable.

## Notes for the G1
- From FieldAI notes: "Press L2 + R2 simultaneously. Note: You may need to press this combination multiple times to ensure the motion
controller is terminated." This implies that the Motion Control Service (possibly via the Motion Switcher Client) is the one we need to deactivate in order to gain low-level (joint-level control).
- It **is** possible to re-enable the Motion Control Service and regain high-level control, since the provided remote controller with the G1 **can** accomplish this.
- From FieldAI notes: "Avoid Command Conflicts: When programming via the SDK, always ensure the robot is in Development Mode (Step 9) to avoid
the internal motion control program interfering with your custom commands." The interface internally contains a state machine and multiplexer to safeguard against exactly this problem.
- **Damping mode is not available in development/debug mode**. We must transition to high-level mode (if we're not in it already) before activating damping or risk damage to the G1.

## TODO:
- Figure out all low-level sdk commands required - In Progress
- Wrap all required low-level sdk commands - In Progress
- Expose wrapped sdk commands via the appropriate modes - In Progress
- Add pubs/subs for the sdk wrapper
- Add pubs/subs for the vector stack
- Build optimizations
- Send a TTS message when the battery is low (stretch)
- Investigate whether or not we need to disable ai_sport via the RobotStateInterface for low-level control instead of/along with the using the MotionSwitcherClient

## SDK Functionalities Needed
- Send velocity commands
- Send TTS commands
- Enter damping mode
- Gain high-level control
- Gain low-level control
- Regain high-level control
- Command motor gains
- Send low-level torque commands