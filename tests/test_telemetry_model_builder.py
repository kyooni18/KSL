"""Check saved shroud visibility when assembling the telemetry mesh."""
from Tools.build_stsn_web_model import Node, jettisoned_transforms


def _module(name, **entries):
    return Node("MODULE", {"name": [name], **{k: [str(v)] for k, v in entries.items()}})


def test_jettisoned_vector_fairing_is_hidden_but_not_nozzle():
    cfg = Node("PART", {}, [_module("ModuleJettison", jettisonName="Fairing")])
    saved = Node("PART", {}, [_module("ModuleJettison", isJettisoned="True", activejettisonName="Fairing")])
    assert jettisoned_transforms(cfg, saved) == {"Fairing"}
    assert "Nozzle" not in jettisoned_transforms(cfg, saved)


def test_attached_vector_fairing_remains_visible():
    cfg = Node("PART", {}, [_module("ModuleJettison", jettisonName="Fairing")])
    saved = Node("PART", {}, [_module("ModuleJettison", isJettisoned="False", activejettisonName="Fairing")])
    assert jettisoned_transforms(cfg, saved) == set()
    assert jettisoned_transforms(cfg, Node("PART")) == set()


def test_jettisoned_transform_uses_config_name_if_missing_from_save():
    cfg = Node("PART", {}, [_module("ModuleJettison", jettisonName="Fairing")])
    saved = Node("PART", {}, [_module("ModuleJettison", isJettisoned="True")])
    assert jettisoned_transforms(cfg, saved) == {"Fairing"}
