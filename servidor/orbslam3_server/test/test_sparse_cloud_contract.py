from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1] / 'src/global_map_server.cpp'
).read_text(encoding='utf-8')


def test_sparse_cloud_keeps_score_and_stable_identity_without_rgb():
    assert 'cloud.point_step = 36;' in SOURCE
    assert 'MakePointField("score", 12' in SOURCE
    assert 'MakePointField("drone_id", 16' in SOURCE
    assert '"map_epoch_low", 20' in SOURCE
    assert '"map_epoch_high", 24' in SOURCE
    assert '"local_mp_id_low", 28' in SOURCE
    assert '"local_mp_id_high", 32' in SOURCE
    assert 'MakePointField("rgb"' not in SOURCE
    assert 'ScoreRgb(' not in SOURCE
    assert 'rgb_field=false' in SOURCE
