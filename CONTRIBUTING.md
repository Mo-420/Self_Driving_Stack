# Contributing to WaveShare Robotaxi

Thank you for your interest in contributing! Please follow these guidelines.

## Branching Model
- **main** – stable, deployable branch
- **feature/<name>** – new features
- **fix/<name>** – bug fixes

## Development Setup
```bash
# Clone and build
git clone https://github.com/Mo-420/Self_Driving_Stack.git WaveShare
cd WaveShare/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Coding Standards
- Python 3.10+
- Keep `flake8` happy: `flake8 ros2_ws/src`
- Format with `black` (line length = 88)

## Testing
```bash
colcon test
colcon test-result --verbose
```

CI will run `colcon build`, linter, and tests on every PR.

## Commit Messages
Use Conventional Commits, e.g.
```
feat: add lidar obstacle layer
fix: handle empty waypoint list
```

## Pull Request Checklist
- [ ] Feature/Fix implemented
- [ ] Linter passes
- [ ] Tests added/updated
- [ ] Documentation updated (README or docs/) 