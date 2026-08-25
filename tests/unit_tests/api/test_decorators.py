# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Tests for jiuwensymbiosis.api.decorators."""

from __future__ import annotations

import typing
from typing import Literal

import pytest

from jiuwensymbiosis.api.actions import implements
from jiuwensymbiosis.api.decorators import (
    ActionSpec,
    ToolMeta,
    _annotation_to_schema,
    _schema_from_signature,
)


class TestAnnotationToSchema:
    def test_int(self):
        assert _annotation_to_schema(int) == {"type": "integer"}

    def test_float(self):
        assert _annotation_to_schema(float) == {"type": "number"}

    def test_str(self):
        assert _annotation_to_schema(str) == {"type": "string"}

    def test_bool(self):
        assert _annotation_to_schema(bool) == {"type": "boolean"}

    def test_list_int(self):
        result = _annotation_to_schema(list[int])
        assert result == {"type": "array", "items": {"type": "integer"}}

    def test_dict(self):
        assert _annotation_to_schema(dict) == {"type": "object"}

    @pytest.mark.parametrize(
        "annotation",
        [typing.Optional.__getitem__(float), float | None],
        ids=["typing-optional", "pep604"],
    )
    def test_optional_float(self, annotation):
        assert _annotation_to_schema(annotation) == {"type": "number"}

    def test_literal(self):
        result = _annotation_to_schema(Literal["a", "b"])
        assert "enum" in result or result == {}
        if "enum" in result:
            assert set(result["enum"]) == {"a", "b"}

    def test_empty_annotation(self):
        from inspect import Parameter

        assert _annotation_to_schema(Parameter.empty) == {}

    def test_any(self):
        from typing import Any

        assert _annotation_to_schema(Any) == {}


class TestSchemaFromSignature:
    def test_simple_function(self):
        def f(x: int, y: float, z: str):
            pass

        schema = _schema_from_signature(f)
        assert schema["type"] == "object"
        assert "x" in schema["properties"]
        assert schema["properties"]["x"] == {"type": "integer"}
        assert schema["properties"]["y"] == {"type": "number"}
        assert "x" in schema["required"]
        assert "x" in schema["required"]

    def test_default_values(self):
        def f(x: int, y: float = 1.0, z: str | None = None):
            pass

        schema = _schema_from_signature(f)
        assert schema["properties"]["y"]["default"] == 1.0
        assert "x" in schema["required"]
        assert "y" not in schema["required"]

    def test_self_excluded(self):
        def f(self, x: int):
            pass

        schema = _schema_from_signature(f)
        assert "self" not in schema["properties"]

    def test_kwargs_excluded(self):
        def f(x: int, **kwargs):
            pass

        schema = _schema_from_signature(f)
        assert "kwargs" not in schema["properties"]


class TestToolMetaCarriesTheSpec:
    """``ToolMeta`` holds its ``ActionSpec`` instead of copying it.

    That is the whole reason the contract fields are declared in one place: a reader
    of ``meta.requires`` and a reader of ``SPEC.requires`` cannot be looking at two
    different answers.
    """

    def test_the_contract_is_the_spec_not_a_copy(self):
        spec = ActionSpec(name="my_tool", description="test tool", tags=("motion",),
                          requires=("payload.clear",), capability="vision.detection")

        @implements(spec)
        def my_tool(self, x: float) -> None:
            pass

        meta = my_tool.__tool_meta__
        assert isinstance(meta, ToolMeta)
        assert meta.spec is spec
        assert (meta.name, meta.description, meta.capability) == ("my_tool", "test tool", "vision.detection")
        assert meta.requires == ("payload.clear",)
        assert meta.tags == ["motion"]  # a list, because that is what the rails read

    def test_input_params_come_from_this_body_signature(self):
        @implements(ActionSpec(name="my_tool", description="d"))
        def my_tool(self, x: float, y: float = 0.0) -> None:
            pass

        params = my_tool.__tool_meta__.input_params
        assert params["type"] == "object"
        assert {"x", "y"} <= set(params["properties"])

    def test_a_declared_param_list_is_what_gets_advertised(self):
        """A body may take more than the contract promises; only the contract is advertised,
        so no plan comes to depend on something the next robot lacks."""

        @implements(ActionSpec(name="my_tool", description="d", params=("x",)))
        def my_tool(self, x: float, body_only_knob: float = 0.0) -> None:
            pass

        assert set(my_tool.__tool_meta__.input_params["properties"]) == {"x"}

    def test_param_schema_refines_what_the_signature_cannot_say(self):
        spec = ActionSpec(
            name="my_tool", description="d", params=("pose",),
            param_schema={"pose": {"type": "object", "properties": {"x": {"type": "number"}}}},
        )

        @implements(spec)
        def my_tool(self, pose: dict) -> None:
            pass

        assert my_tool.__tool_meta__.input_params["properties"]["pose"]["properties"] == {"x": {"type": "number"}}
