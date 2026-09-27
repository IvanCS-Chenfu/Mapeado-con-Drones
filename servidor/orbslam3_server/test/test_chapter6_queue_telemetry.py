from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SERVER_SOURCE = PACKAGE_ROOT / "src" / "global_map_server.cpp"


def test_chapter6_queue_telemetry_is_opt_in_and_passive():
    source = SERVER_SOURCE.read_text(encoding="utf-8")

    assert 'declare_parameter<bool>(\n      "chapter6_queue_telemetry_enabled", false)' in source
    assert 'create_wall_timer(\n          std::chrono::milliseconds(chapter6_queue_telemetry_period_ms_)' in source
    assert '[C6-QUEUE-SAMPLE]' in source
    assert 'primary_pending=%zu secondary_pending=%zu' in source
    assert 'secondary_critical=%zu secondary_maintenance=%zu' in source
    assert 'optimization_active=%s' in source
    assert 'backpressure_active=%s' in source


def test_chapter6_keyframe_pose_telemetry_is_opt_in_and_uses_incremental_views():
    source = SERVER_SOURCE.read_text(encoding="utf-8")

    assert 'declare_parameter<bool>(\n      "chapter6_keyframe_pose_telemetry_enabled", false)' in source
    assert 'EmitChapter6KeyframePoseSamples(build, source);' in source
    assert '[C6-KF-POSE]' in source
    assert 'for (const auto & keyframe : build.delta_keyframe_upserts)' in source
    assert 'backend_.GetRawKeyFrame(keyframe.keyframe_id)' in source
