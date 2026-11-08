"""Plain-text operator task -> structured task: the mock Module 1 parser.
Pure Python (no rclpy), unit-tested in tests/unit/test_task_text.py.

A keyword grammar, not language understanding: one task per text. The verb
picks the type (patrol > delivery > go to: "go to the storage and bring the
parcel to lab 2" is a delivery). Locations are named by registry id or label,
spelled loosely ("lab_2", "Laboratory 2", "lab 2", "Lab2", "laboratory two").
A delivery visits pickup then drop-off, so "to X from Y" is reordered. Text
that does not read as exactly one task raises ParseError, whose message is
the reason Module 1 reports with ERROR_PARSE: guessing would send a robot to
the wrong place.
"""
import re
from dataclasses import dataclass

GO_TO_WORDS = {'go', 'goto', 'drive', 'navigate', 'move', 'head', 'return', 'visit', 'come'}
DELIVERY_WORDS = {'bring', 'deliver', 'carry', 'transport', 'fetch', 'take'}
PATROL_WORDS = {'patrol', 'patrolling'}
PRIORITY_LEVELS = ('low', 'normal', 'high', 'urgent')     # ascending
URGENT_WORDS = {'urgent', 'urgently', 'asap', 'immediately'}
REPEAT_WORDS = {'times', 'loops', 'rounds', 'laps', 'x'}  # "<n> times"
REPEAT_ADVERBS = {'once': 1, 'twice': 2, 'thrice': 3}
# capability a robot needs per task type (agent capability vocabulary)
CAPABILITIES = {'delivery': ('deliver',), 'patrol': ('patrol',)}

_ARTICLES = {'a', 'an', 'the'}
_SYNONYMS = {'laboratory': 'lab', 'one': '1', 'two': '2', 'three': '3', 'four': '4',
             'five': '5', 'six': '6', 'seven': '7', 'eight': '8', 'nine': '9', 'ten': '10'}


class ParseError(ValueError):
    """The text is not exactly one task; str(error) says why."""


@dataclass(frozen=True)
class ParsedTask:
    """Task fields in the mock M1 script vocabulary (type, locations, ...)."""
    type: str                          # 'go_to' | 'delivery' | 'patrol'
    locations: tuple                   # registry ids in visiting order
    priority: str = 'normal'           # one of PRIORITY_LEVELS
    patrol_loops: int = 1
    required_capabilities: tuple = ()


def words(text):
    """Normalized words: lower case, punctuation and articles dropped,
    'lab_2' / 'Lab2' -> 'lab', '2', synonyms and number words mapped."""
    text = re.sub(r'[^a-z0-9]+', ' ', text.lower())
    text = re.sub(r'(?<=[a-z])(?=[0-9])|(?<=[0-9])(?=[a-z])', ' ', text)
    return [_SYNONYMS.get(word, word) for word in text.split() if word not in _ARTICLES]


def parse_task_text(text, locations):
    """text + location registry {id: label} -> ParsedTask; ParseError if the
    text is not exactly one task."""
    tokens = words(text)
    if not tokens:
        raise ParseError('empty task text')
    task_type = _task_type(tokens)
    loops, tokens = _repeat_count(tokens)
    if loops is not None and loops < 1:
        raise ParseError('repeat count must be at least 1')
    if task_type != 'patrol' and loops not in (None, 1):
        raise ParseError('repeat count only applies to patrol')
    mentions = _mentions(tokens, _aliases(locations))
    if not mentions:
        raise ParseError(f'no known location; known: {", ".join(locations)}')
    ids = [location_id for _, location_id in mentions]
    if task_type == 'go_to' and len(ids) != 1:
        raise ParseError(f'go to takes one location, got {", ".join(ids)}')
    if task_type == 'delivery':
        if len(ids) != 2:
            raise ParseError('delivery needs a pickup and a drop-off location, '
                             f'got {", ".join(ids)}')
        second_index = mentions[1][0]
        if tokens[second_index - 1] == 'from':            # "to X from Y": Y is the pickup
            ids.reverse()
    return ParsedTask(task_type, tuple(ids), _priority(tokens), loops or 1,
                      CAPABILITIES.get(task_type, ()))


def _task_type(tokens):
    present = set(tokens)
    patrol, delivery = present & PATROL_WORDS, present & DELIVERY_WORDS
    if patrol and delivery:
        raise ParseError('both patrol and delivery requested; send one task per text')
    if patrol:
        return 'patrol'
    if delivery:
        return 'delivery'
    if present & GO_TO_WORDS:
        return 'go_to'
    raise ParseError('no task verb: expected go to, bring/deliver or patrol')


def _repeat_count(tokens):
    """(loop count or None, tokens without it). The count is removed before
    locations are matched, so in 'patrol the dock 2 times' the 2 is no dock."""
    counts, rest = [], []
    index = 0
    while index < len(tokens):
        word = tokens[index]
        following = tokens[index + 1] if index + 1 < len(tokens) else ''
        if word in REPEAT_ADVERBS:
            counts.append(REPEAT_ADVERBS[word])
        elif word.isdigit() and following in REPEAT_WORDS:
            counts.append(int(word))
            index += 1                                    # the repeat word goes too
        else:
            rest.append(word)
        index += 1
    if len(counts) > 1:
        raise ParseError('repeat count given twice')
    return (counts[0] if counts else None), rest


def _aliases(locations):
    """Word tuple -> location ids it names (more than one id = ambiguous)."""
    aliases = {}
    for location_id, label in locations.items():
        for name in (location_id, label):
            key = tuple(words(name))
            if key:
                aliases.setdefault(key, set()).add(location_id)
    return aliases


def _mentions(tokens, aliases):
    """[(token index, location id)] in text order; the longest alias wins
    ('charging dock 2' is dock_2, not dock). A word that only starts the
    names of several locations ('lab' of lab 1 / lab 2) is ambiguous."""
    heads = {}
    for alias, ids in aliases.items():
        if len(alias) > 1:
            heads.setdefault(alias[0], set()).update(ids)
    mentions, index = [], 0
    while index < len(tokens):
        size, ids = _longest_alias(tokens, index, aliases)
        if ids is None:
            if len(heads.get(tokens[index], ())) > 1:
                raise _ambiguous(tokens[index], heads[tokens[index]])
            index += 1
            continue
        if len(ids) > 1:
            raise _ambiguous(' '.join(tokens[index:index + size]), ids)
        mentions.append((index, next(iter(ids))))
        index += size
    return mentions


def _longest_alias(tokens, index, aliases):
    """(word count, location ids) of the longest alias starting at
    tokens[index]; (0, None) when none does."""
    longest = max(map(len, aliases), default=0)
    for size in range(min(longest, len(tokens) - index), 0, -1):
        ids = aliases.get(tuple(tokens[index:index + size]))
        if ids:
            return size, ids
    return 0, None


def _ambiguous(name, ids):
    ids = sorted(ids)
    return ParseError(f"ambiguous location '{name}': {', '.join(ids[:-1])} or {ids[-1]}")


def _priority(tokens):
    """Highest level named: 'urgent' / 'asap' / ..., or a level next to
    'priority' ('high priority', 'priority low'); default normal."""
    levels = set()
    for index, word in enumerate(tokens):
        if word in URGENT_WORDS:
            levels.add('urgent')
        if word == 'priority':
            neighbours = tokens[max(index - 1, 0):index] + tokens[index + 1:index + 2]
            levels.update(neighbour for neighbour in neighbours if neighbour in PRIORITY_LEVELS)
    return max(levels, key=PRIORITY_LEVELS.index, default='normal')
