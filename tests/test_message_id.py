from typing import Dict, Tuple

from franka_control_client.core.message import MsgID


def build_msgid_registry() -> Dict[int, Tuple[str, str]]:
    """Build registry by scanning all MsgID subclasses."""
    registry: Dict[int, Tuple[str, str]] = {}

    for subclass in MsgID.__subclasses__():
        for member in subclass:
            if member.value in registry:
                prev_cls, prev_name = registry[member.value]
                raise ValueError(
                    f"Duplicate MsgID value {member.value}: "
                    f"{subclass.__name__}.{member.name} conflicts with {prev_cls}.{prev_name}"
                )
            registry[member.value] = (subclass.__name__, member.name)

    return registry


def check_for_duplicates(registry: Dict[int, Tuple[str, str]]) -> None:
    """Validate there are no duplicate IDs."""
    seen: Dict[int, str] = {}
    for val, (cls_name, name) in sorted(registry.items()):
        if val in seen:
            raise ValueError(
                f"Duplicate ID {val}: {cls_name}.{name} and {seen[val]}"
            )
        seen[val] = f"{cls_name}.{name}"
    print(f"[MsgID] ✅ No duplicates found among {len(seen)} IDs.")


def dump_msgid_table(registry: Dict[int, Tuple[str, str]]) -> None:
    """Pretty-print all registered MsgIDs."""
    print("\n[MsgID Table]")
    print(f"{'Dec':<6} | {'Class':<20} | {'Name'}")
    print("-" * 50)
    for val, (cls_name, name) in sorted(registry.items()):
        print(f"{val:<6} | {cls_name:<20} | {name}")


def test_msgid_unique_and_table():
    registry = build_msgid_registry()
    check_for_duplicates(registry)
    dump_msgid_table(registry)
