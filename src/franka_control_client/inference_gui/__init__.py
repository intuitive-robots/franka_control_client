"""GUI-driven remote policy inference for the Franka Panda.

Two processes, one on each machine:

* Policy PC:  ``python -m franka_control_client.inference_gui.policy_server``
  wraps any policy behind the length-prefixed pickle socket protocol of
  :mod:`franka_control_client.policy_inference.policy_server_client`.
* Robot PC:   ``python -m franka_control_client.inference_gui.app``
  runs the control pair + inference loop and serves a browser dashboard.

Nothing in this package's ``__init__`` imports pyzlc / torch, so the policy
server only needs numpy on the policy PC.
"""
