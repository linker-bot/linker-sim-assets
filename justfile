python := env_var_or_default("PYTHON", "python3")

# Install the authoring extra before running the asset regression suite.
test:
    PYTHONPATH=src {{python}} -m unittest discover -s tests -v

check-drift:
    PYTHONPATH=src {{python}} -m unittest discover -s tests -p test_collision_exclusions.py -k test_all_assets_pass_full_validation

# Regenerate named base collision geometry and verify its conservative envelope.
build-base-collisions:
    {{python}} scripts/build_base_collisions.py a7_torso a7_lite_torso p7_torso bench_table

check-base-collisions:
    {{python}} scripts/build_base_collisions.py --check a7_torso a7_lite_torso p7_torso bench_table
