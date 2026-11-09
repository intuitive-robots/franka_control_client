"""Browser GUI for data collection on the Franka (robot PC side).

    python -m franka_control_client.data_collection_gui.app --config configs/data_collection_gui.json
    # open http://localhost:8090

Three tabs:

* Collect   GELLO / VR (Meta Quest 3) teleoperation, kinesthetic demos, teach & replay;
            saves the IRL raw format and/or a LeRobot dataset (chosen fps).
* Convert   import an IRL recording folder and convert it to a LeRobot dataset.
* View      open a LeRobot dataset: synced camera videos + state/action plots per episode.

``irl_format`` and ``irl_to_lerobot`` (also a CLI) do not need the robot or pyzlc.
"""
