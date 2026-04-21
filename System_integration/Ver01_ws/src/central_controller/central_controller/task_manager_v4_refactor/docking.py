try:
    from nav2_msgs.action import DockRobot  # type: ignore[attr-defined]

    DOCKROBOT_IMPORT_ERROR = None
except Exception as nav2_import_error:
    try:
        from opennav_docking_msgs.action import DockRobot  # pyright: ignore[reportMissingImports]

        DOCKROBOT_IMPORT_ERROR = None
    except Exception as docking_msgs_import_error:
        DockRobot = None  # type: ignore
        DOCKROBOT_IMPORT_ERROR = (
            "Failed to import DockRobot from both nav2_msgs.action and "
            f"opennav_docking_msgs.action: nav2_msgs={nav2_import_error}, "
            f"opennav_docking_msgs={docking_msgs_import_error}"
        )

