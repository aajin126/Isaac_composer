"""Small OpenAI-compatible HTTP client for the configured Cosmos server."""

from __future__ import annotations

import base64
import json
import os
from pathlib import Path
import re
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from ..text import normalize_json_typography

SYSTEM = (
    "You plan and review activities of HUMAN PEDESTRIANS ON FOOT inside the actual scene shown. "
    "Only people may move, walk, wait, look around or queue. Existing objects are static background. "
    "Do not plan operating vehicles, transporting goods or adding facilities. "
    "Imagine people's activities within THIS environment, not an imagined generic environment. "
    "All JSON keys and values must be English ASCII. Return only the requested JSON object, "
    "without markdown or explanations."
)


class CosmosError(RuntimeError):
    def __init__(self, message, response_text=None):
        super().__init__(message)
        self.response_text = response_text


def parse_json_answer(content):
    # Cosmos deployments may expose reasoning separately or wrap the final
    # answer in tags. Never parse an unfinished reasoning block as a plan.
    text = content.strip()
    if "<think>" in text:
        if "</think>" not in text:
            raise CosmosError("Cosmos returned unfinished reasoning instead of a JSON answer")
        text = text.split("</think>", 1)[1].strip()
    match = re.search(r"<answer>(.*?)</answer>", text, re.S)
    if match:
        text = match.group(1).strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*|\s*```$", "", text)
    try:

        def unique_keys(pairs):
            result = {}
            for key, item in pairs:
                if key in result:
                    raise ValueError("Duplicate JSON keys")
                result[key] = item
            return result

        value = normalize_json_typography(json.loads(text, object_pairs_hook=unique_keys))
    except json.JSONDecodeError as error:
        raise CosmosError(
            f"Cosmos did not return one valid JSON object: {error.msg} "
            f"at line {error.lineno}, column {error.colno}"
        ) from error
    except (ValueError, TypeError) as error:
        raise CosmosError("Cosmos did not return one valid JSON object") from error
    if isinstance(value, list) and len(value) == 1 and isinstance(value[0], dict):
        value = value[0]  # One unambiguous document, with an unnecessary array wrapper.
    if not isinstance(value, dict):
        raise CosmosError("Cosmos answer must be a JSON object")
    return value


class CosmosClient:
    def __init__(
        self,
        base_url="http://172.17.0.2:8000",
        model=None,
        timeout_s=120,
        max_tokens=4096,
        structured_output=False,
        response_dir=None,
        seed=None,
    ):
        self.base_url = base_url.rstrip("/")
        if not self.base_url.endswith("/v1"):
            self.base_url += "/v1"
        self.model, self.timeout_s, self.max_tokens = model, timeout_s, max_tokens
        self.structured_output = structured_output
        self.response_dir = Path(response_dir) if response_dir else None
        self.request_index = 0
        self.seed = seed

    def _request(self, path, body=None):
        headers = {"Content-Type": "application/json"}
        key = os.environ.get("COSMOS_API_KEY")
        if key:
            headers["Authorization"] = f"Bearer {key}"
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = Request(self.base_url + path, data=data, headers=headers)
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                return json.load(response)
        except (HTTPError, URLError, TimeoutError) as error:
            # No keys or full multimodal request bodies in error output.
            code = getattr(error, "code", None)
            raise CosmosError(
                f"Cosmos HTTP request failed; status={code}; check endpoint and model"
            ) from error

    def discover_model(self):
        if self.model:
            return self.model
        models = self._request("/models").get("data", [])
        candidates = [m["id"] for m in models if "cosmos" in m.get("id", "").lower()]
        if len(candidates) != 1:
            raise CosmosError(
                "Specify --model when the server does not expose exactly one Cosmos model"
            )
        self.model = candidates[0]
        return self.model

    def complete(self, instruction, payload, images=(), schema=None, temperature=0.3):
        content = []
        for path in images:
            encoded = base64.b64encode(Path(path).read_bytes()).decode("ascii")
            content.append(
                {"type": "image_url", "image_url": {"url": "data:image/png;base64," + encoded}}
            )
        content.append({"type": "text", "text": instruction + "\nINPUT:\n" + json.dumps(payload)})
        body = {
            "model": self.discover_model(),
            "messages": [
                {
                    "role": "system",
                    "content": SYSTEM,
                },
                {"role": "user", "content": content},
            ],
            "temperature": temperature,
            "top_p": 0.9 if temperature > 0.5 else 0.8,
            "top_k": 20,
            "repetition_penalty": 1.1,
            "max_tokens": self.max_tokens,
        }
        if self.seed is not None:
            body["seed"] = self.seed + self.request_index
        if self.structured_output:
            body["messages"][0]["content"] = SYSTEM + " Match the provided output schema."
            body["response_format"] = (
                {
                    "type": "json_schema",
                    "json_schema": {
                        "name": "scenario_response",
                        "schema": schema,
                        "strict": True,
                    },
                }
                if schema
                else {"type": "json_object"}
            )
        response = self._request("/chat/completions", body)
        choice = response["choices"][0]
        self.request_index += 1
        if self.response_dir:
            self.response_dir.mkdir(parents=True, exist_ok=True)
            # Text diagnostics preserve malformed model output without presenting
            # it as an accepted English JSON artifact. Never save credentials.
            (self.response_dir / f"response_{self.request_index:04d}.txt").write_text(
                choice["message"].get("content") or "", encoding="utf-8"
            )
        if choice.get("finish_reason") == "length":
            raise CosmosError("Cosmos response hit max_tokens; no partial plan was accepted")
        raw = choice["message"].get("content") or ""
        try:
            return parse_json_answer(raw)
        except CosmosError as error:
            # Retain the rejected answer for a bounded format repair. Raw text
            # remains diagnostic input, never an accepted or executable plan.
            raise CosmosError(str(error), response_text=raw) from error
