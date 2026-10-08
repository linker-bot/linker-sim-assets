python := env_var_or_default("PYTHON", "python3")

# Install the authoring extra before running the asset regression suite.
test:
    PYTHONPATH=src {{python}} -m unittest discover -s tests -v

check-drift:
    PYTHONPATH=src {{python}} -m unittest discover -s tests -p test_collision_exclusions.py -k test_all_assets_pass_full_validation
