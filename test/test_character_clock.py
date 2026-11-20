"""Manual clock ownership and per-character simulation-time playback."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from isaac_composer.crowd.runtime.clock import CharacterClock


def test_character_rates_are_applied_before_the_manager_publishes_the_frame():
    settings = Mock()
    settings.get.return_value = False
    clock = CharacterClock(settings)
    settings.set_bool.assert_called_once_with("/animGraph/manualUpdate", True)
    interface = Mock()
    interface.initialize.return_value = True
    clock.start(interface, 42)
    characters = [Mock(), Mock()]
    people = [
        SimpleNamespace(character_graph=characters[0], _animation_playback_rate=1.6),
        SimpleNamespace(character_graph=characters[1], _animation_playback_rate=1.0),
        SimpleNamespace(character_graph=None),
    ]
    clock.advance(0.0, people)
    clock.advance(0.02, people)
    assert characters[0].update.call_count == 1
    assert characters[0].update.call_args.args[0] == pytest.approx(0.032)
    characters[1].update.assert_called_once_with(0.02)
    interface.update.assert_called_once_with(0.02)  # Publishes the explicitly updated poses.
    clock.close()
    clock.close()
    interface.shutdown.assert_called_once()
    assert settings.set_bool.call_args.args == ("/animGraph/manualUpdate", False)


def test_failed_initialization_is_reported_and_previous_setting_is_restored():
    settings = Mock()
    settings.get.return_value = True
    clock = CharacterClock(settings)
    interface = Mock()
    interface.initialize.return_value = False
    with pytest.raises(RuntimeError, match="initialize"):
        clock.start(interface, 42)
    clock.close()
    interface.shutdown.assert_not_called()
    assert settings.set_bool.call_args.args == ("/animGraph/manualUpdate", True)
