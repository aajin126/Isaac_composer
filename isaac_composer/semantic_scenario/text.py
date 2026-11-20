"""Normalize English typography without translating or dropping model content."""

_PUNCTUATION = str.maketrans(
    {
        "\u2018": "'",
        "\u2019": "'",
        "\u201a": "'",
        "\u201b": "'",
        "\u201c": '"',
        "\u201d": '"',
        "\u201e": '"',
        "\u201f": '"',
        "\u2010": "-",
        "\u2011": "-",
        "\u2012": "-",
        "\u2013": "-",
        "\u2014": "-",
        "\u2212": "-",
        "\u2026": "...",
        "\u00a0": " ",
        "\u202f": " ",
    }
)


def normalize_json_typography(value):
    """Keep words and identifiers intact; standardize common punctuation only."""
    if isinstance(value, str):
        return value.translate(_PUNCTUATION)
    if isinstance(value, list):
        return [normalize_json_typography(item) for item in value]
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            normalized = normalize_json_typography(key)
            if normalized in result:
                raise ValueError("Duplicate JSON keys after typography normalization")
            result[normalized] = normalize_json_typography(item)
        return result
    return value
