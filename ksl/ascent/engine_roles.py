"""Engine identity helpers shared by the STS ascent and orbit programs."""


def is_oms_engine(engine):
    """Recognize current Terrier OMS engines and the legacy Viking/AJ10 pair."""
    part = engine.part
    identity = f"{getattr(part, 'name', '')} {getattr(part, 'title', '')}".casefold()
    return any(token in identity for token in (
        'terrier', 'viking', 'benjee10.aj10.v2', 'orbital manoeuvr',
        'orbital maneuver', ' oms',
    ))
