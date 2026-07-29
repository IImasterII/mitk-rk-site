#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ДЕМОНСТРАЦИОННАЯ ПРОГРАММА ПОКАЗЫАЮЩАЯ ПРИНЦИП ПОСТРОЕНИЯ РАБОЧЕГО КОЛЕСА(УПРОЩЁННАЯ МОДЕЛЬ)
IMPELLER GENERATOR D200 — параметрическое рабочее колесо центробежного компрессора. 

Основной результат программы — одно герметичное тело в STEP/STL/3MF. Лопатки
строятся как настоящие пространственные линейчатые поверхности: сначала
вычисляются hub- и shroud-профили, затем соответствующие точки соединяются
прямыми образующими. Укороченные splitter blades являются производными от
основной лопатки и располагаются в заданной доле межлопаточного шага.

Установка:
    python -m pip install cadquery trimesh manifold3d networkx

Обычный запуск:
    python impeller_generator_D200_PA12CF10.py

Параметры для редактирования находятся сразу ниже, в USER_PARAMETERS.
Дополнительные примеры:
    python impeller_generator_D200_PA12CF10.py --quality fine
    python impeller_generator_D200_PA12CF10.py --output ./my_impeller
    python impeller_generator_D200_PA12CF10.py --set geometry.D2_mm=80 blades.main_blades=12
    python impeller_generator_D200_PA12CF10.py --config my_config.json
    python impeller_generator_D200_PA12CF10.py --check-only
    python impeller_generator_D200_PA12CF10.py --optimize
    python impeller_generator_D200_PA12CF10.py --optimize-only

Инженерные ограничения:
* mean-line аэродинамика и аналитическая прочность являются screening-оценками;
* перед вращательными испытаниями нужны CFD, FEA, modal/Campbell, overspeed proof
  test и динамическая балансировка;
* STEP экспортируется как единое булево-объединённое B-Rep-тело. Корни лопаток
  имеют гарантированное заглубление в ступицу, поэтому в CAD отсутствуют
  скрытые root-cap кромки и нулевые/касательные контакты.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import sys
import time
import traceback
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

# =============================================================================
#                         ПАРАМЕТРЫ ДЛЯ ИЗМЕНЕНИЯ
# =============================================================================
# Все размеры — мм, углы — градусы, обороты — об/мин.
# Базовый вариант восстановлен под исходные параметры проекта: D2=200 мм,
# N=12000 об/мин, PA12-CF10, 9 полноразмерных лопаток и 9 сплиттеров.

USER_PARAMETERS: dict[str, Any] = {
    "operating": {
        "rpm": 12000.0,
        "p01_pa": 101325.0,
        "t01_k": 293.15,
        "eta_tt": 0.72,
        "flow_coefficient_phi2": 0.28,
        "blockage_inlet": 0.95,
        "blockage_exit": 0.88,
        "prewhirl_ratio": 0.0,
    },
    "geometry": {
        "D2_mm": 200.0,
        "D1_shroud_mm": 110.0,
        "D1_hub_mm": 40.0,
        "bore_diameter_mm": 30.0,
        "b2_mm": 12.0,
        "axial_length_mm": 45.0,
        # Увеличиваем толщину заднего диска для жесткости на шпинделе
        "backplate_thickness_mm": 8.0,
        # Общий радиальный припуск наружной кромки диска и всех лопаток.
        # D2 остаётся чистовым диаметром 200 мм, а единая заготовка Ø202 мм
        # вручную подрезается по диску и выходным кромкам до Ø200 мм.
        "outer_edge_finish_allowance_radial_mm": 1.0,
        "nose_extra_height_mm": 1.5,
        "hub_p1_dr_fraction": 0.020,
        "hub_p1_drop_fraction": 0.030,
        "hub_p2_dr_fraction": 0.240,
        "hub_p2_z_fraction": 0.090,
        "shroud_p1_dr_fraction": 0.030,
        "shroud_p1_drop_fraction": 0.200,
        "shroud_p2_dr_fraction": 0.150,
        "shroud_p2_rise_fraction": 0.150,
    },
    "blades": {
        "main_blades": 9,
        "add_splitters": True,
        "splitter_start_fraction": 0.505,
        "splitter_pitch_fraction": 0.50,
        "splitter_thickness_scale": 0.90,
        "splitter_slip_effectiveness": 0.50,
        "backsweep_from_radial_deg": 35.0,
        "inlet_angle_mode": "auto",
        "manual_beta1_hub_deg": 54.0,
        "manual_beta1_shroud_deg": 27.0,
        "incidence_deg": 0.0,
        "lean_le_deg": 4.0,
        "lean_te_deg": -2.0,
        "wrap_scale": 1.00,
        "loading_bias": 0.00,
        # --- АДАПТАЦИЯ ПОД 3D ПЕЧАТЬ ---
        # Утолщены кромки для печатаемости (минимум 1.2 мм для сопла 0.4)
        "t_le_hub_mm": 2.80,
        "t_max_hub_mm": 4.50,
        "t_te_hub_mm": 1.50,  # было 1.20
        "t_le_shroud_mm": 2.00,
        "t_max_shroud_mm": 3.60,
        "t_te_shroud_mm": 1.20, # было 1.00
        "max_thickness_location_hub": 0.34,
        "max_thickness_location_shroud": 0.38,
        "leading_edge_ellipse_ratio_hub": 4.0,
        "leading_edge_ellipse_ratio_shroud": 6.0,
        # Углубление корня в ступицу увеличено для защиты от отрыва по слоям Z
        "root_embed_mm": 1.50, # было 0.80
        "root_thickness_factor": 1.50, # было 1.20 - усиление галтели
        # Полноразмерная лопатка у hub начинается чуть дальше входного шва
        # ступицы. Tip/аэродинамическая LE остаются на расчётном месте.
        "main_root_le_trim_fraction": 0.040,
        "chord_samples": 151,
        "span_sections": 11,
        "leading_edge_arc_points": 9,
    },
    "manufacturing": {
        "material": "PA12-CF10 (FDM, conservative)",
        # Повышенный коэффициент запаса для полимеров при циклических нагрузках
        "required_safety_factor": 3.00, # было 2.00
        # Учет микропустот FDM-печати между линиями экструзии
        "stress_concentration_factor": 2.00, # было 1.60
        "keyway_enabled": False,
        "keyway_width_mm": 5.0,
        "keyway_depth_mm": 2.3,
        # Компенсация усадки PA12-CF10 (калибруется под конкретный принтер)
        "scale_xy": 1.0030, 
        "scale_z": 1.0050,
    },
    "output": {
        "output_directory": "impeller_D200_PA12CF10_optimized",
        "mesh_quality": "fine",
        "export_step_reference_assembly": True,
        # По умолчанию сохраняется только единая fused STEP-модель.
        # Возможности остальных экспортов остаются в коде и включаются через
        # JSON/--set при необходимости.
        "export_component_steps": False,
        "export_stl": False,
        "export_3mf": False,
        "export_glb": False,
        "export_smooth_visual_glb": False,
        "export_csv": False,
        "export_metadata": False,
        "mesh_simplify_tolerance_mm": 0.003,
        "reload_and_validate_stl": False,
    }
}

# Автоматическая трёхпрофильная оптимизация. Стендовая точка используется
# только для прогноза по законам подобия; геометрия оптимизируется на design_rpm.
# Диапазон расхода не позволяет математически увеличить напор сведением
# полезной подачи почти к нулю.
OPTIMIZATION_PARAMETERS: dict[str, Any] = {
    "design_rpm": 15000.0,
    "stand_rpm": 3000.0,
    "mass_flow_min_kg_s": 0.20,
    "mass_flow_max_kg_s": 0.40,
    "seed": 42,
    "max_iterations": 28,
    "population_size": 9,
    "profiles": ("max_pressure", "max_efficiency", "balanced"),
}

# =============================================================================
#                               ЗАВИСИМОСТИ
# =============================================================================

try:
    import numpy as np
except Exception as exc:  # pragma: no cover
    raise SystemExit("Не найден numpy: python -m pip install numpy") from exc

try:
    import cadquery as cq
except Exception:  # pragma: no cover
    cq = None  # type: ignore[assignment]

try:
    import trimesh
except Exception:  # pragma: no cover
    trimesh = None  # type: ignore[assignment]

try:
    import manifold3d  # noqa: F401  # активирует engine='manifold'
except Exception:  # pragma: no cover
    manifold3d = None  # type: ignore[assignment]


def require_cad_dependencies() -> None:
    missing: list[str] = []
    if cq is None:
        missing.append("cadquery")
    if trimesh is None:
        missing.append("trimesh")
    if manifold3d is None:
        missing.append("manifold3d")
    if missing:
        raise RuntimeError(
            "Для CAD-экспорта не установлены: " + ", ".join(missing)
            + ". Установка: python -m pip install cadquery trimesh manifold3d networkx"
        )

# =============================================================================
#                       КОНСТАНТЫ, МАТЕРИАЛЫ, QUALITY
# =============================================================================

R_AIR = 287.05
GAMMA_AIR = 1.4
CP_AIR = 1004.7

# Значения ориентировочные и предназначены только для screening.
MATERIALS: dict[str, dict[str, Any]] = {
    "PA12-CF10 (FDM, conservative)": dict(
        density=1060.0,
        # Консервативная база — предел прочности Z-направления после
        # кондиционирования влагой из TDS Fiberon PA12-CF10: 42.1 MPa.
        # Для полимера это не классический металлический предел текучести.
        yield_mpa=42.1,
        E_gpa=1.8066,
        nu=0.35,
        k=1.00,
        strength_basis="wet Z-direction tensile-strength proxy",
        source_note=(
            "Fiberon PA12-CF10 TDS v1.1: density 1.06 g/cm^3; dry XY/Z tensile "
            "77.4/52.2 MPa; wet Z tensile 42.1 MPa. Printed-part properties are anisotropic."
        ),
    ),
    "Al 7075-T6": dict(density=2810.0, yield_mpa=435.0, E_gpa=71.7, nu=0.33, k=1.00,
                        strength_basis="yield strength", source_note="screening reference"),
    "Al 2014-T6": dict(density=2800.0, yield_mpa=345.0, E_gpa=73.0, nu=0.33, k=1.00,
                        strength_basis="yield strength", source_note="screening reference"),
    "Ti-6Al-4V": dict(density=4430.0, yield_mpa=830.0, E_gpa=114.0, nu=0.34, k=1.00,
                       strength_basis="yield strength", source_note="screening reference"),
    "17-4PH H900": dict(density=7800.0, yield_mpa=1000.0, E_gpa=197.0, nu=0.29, k=1.00,
                         strength_basis="yield strength", source_note="screening reference"),
}

QUALITY: dict[str, dict[str, float]] = {
    "draft": dict(chord_scale=0.68, span_sections=7, le_arc=5, hub_linear=0.18, hub_angular=0.14),
    "normal": dict(chord_scale=1.00, span_sections=9, le_arc=7, hub_linear=0.08, hub_angular=0.070),
    # Для D2=200 мм окружная tessellation должна быть существенно плотнее,
    # иначе STL остаётся герметичным, но на hub видны крупные фасеты.
    "fine": dict(chord_scale=1.45, span_sections=13, le_arc=11, hub_linear=0.035, hub_angular=0.035),
}

MIN_ROOT_COMMON_VOLUME_MM3 = 1.0e-3

# =============================================================================
#                              КОНФИГУРАЦИЯ
# =============================================================================


@dataclass(frozen=True)
class OperatingConfig:
    rpm: float
    p01_pa: float
    t01_k: float
    eta_tt: float
    flow_coefficient_phi2: float
    blockage_inlet: float
    blockage_exit: float
    prewhirl_ratio: float


@dataclass(frozen=True)
class GeometryConfig:
    D2_mm: float
    D1_shroud_mm: float
    D1_hub_mm: float
    bore_diameter_mm: float
    b2_mm: float
    axial_length_mm: float
    backplate_thickness_mm: float
    outer_edge_finish_allowance_radial_mm: float
    nose_extra_height_mm: float
    hub_p1_dr_fraction: float
    hub_p1_drop_fraction: float
    hub_p2_dr_fraction: float
    hub_p2_z_fraction: float
    shroud_p1_dr_fraction: float
    shroud_p1_drop_fraction: float
    shroud_p2_dr_fraction: float
    shroud_p2_rise_fraction: float


@dataclass(frozen=True)
class BladeConfig:
    main_blades: int
    add_splitters: bool
    splitter_start_fraction: float
    splitter_pitch_fraction: float
    splitter_thickness_scale: float
    splitter_slip_effectiveness: float
    backsweep_from_radial_deg: float
    inlet_angle_mode: str
    manual_beta1_hub_deg: float
    manual_beta1_shroud_deg: float
    incidence_deg: float
    lean_le_deg: float
    lean_te_deg: float
    wrap_scale: float
    loading_bias: float
    t_le_hub_mm: float
    t_max_hub_mm: float
    t_te_hub_mm: float
    t_le_shroud_mm: float
    t_max_shroud_mm: float
    t_te_shroud_mm: float
    max_thickness_location_hub: float
    max_thickness_location_shroud: float
    leading_edge_ellipse_ratio_hub: float
    leading_edge_ellipse_ratio_shroud: float
    root_embed_mm: float
    root_thickness_factor: float
    main_root_le_trim_fraction: float
    chord_samples: int
    span_sections: int
    leading_edge_arc_points: int


@dataclass(frozen=True)
class ManufacturingConfig:
    material: str
    required_safety_factor: float
    stress_concentration_factor: float
    keyway_enabled: bool
    keyway_width_mm: float
    keyway_depth_mm: float
    scale_xy: float
    scale_z: float


@dataclass(frozen=True)
class OutputConfig:
    output_directory: str
    mesh_quality: str
    export_step_reference_assembly: bool
    export_component_steps: bool
    export_stl: bool
    export_3mf: bool
    export_glb: bool
    export_smooth_visual_glb: bool
    export_csv: bool
    export_metadata: bool
    mesh_simplify_tolerance_mm: float
    reload_and_validate_stl: bool


@dataclass(frozen=True)
class DesignConfig:
    operating: OperatingConfig
    geometry: GeometryConfig
    blades: BladeConfig
    manufacturing: ManufacturingConfig
    output: OutputConfig


def design_from_dict(data: dict[str, Any]) -> DesignConfig:
    return DesignConfig(
        operating=OperatingConfig(**data["operating"]),
        geometry=GeometryConfig(**data["geometry"]),
        blades=BladeConfig(**data["blades"]),
        manufacturing=ManufacturingConfig(**data["manufacturing"]),
        output=OutputConfig(**data["output"]),
    )


def validate_config(cfg: DesignConfig) -> None:
    g, b, o, m = cfg.geometry, cfg.blades, cfg.operating, cfg.manufacturing
    errors: list[str] = []
    if not (g.D2_mm > g.D1_shroud_mm > g.D1_hub_mm > g.bore_diameter_mm + 1.0):
        errors.append("Требуется D2 > D1_shroud > D1_hub > bore + 1 мм.")
    if not (0.0 < g.b2_mm < g.axial_length_mm):
        errors.append("Требуется 0 < b2 < axial_length.")
    if g.backplate_thickness_mm <= 0.5:
        errors.append("Толщина заднего диска должна быть > 0.5 мм.")
    if not (0.0 <= g.outer_edge_finish_allowance_radial_mm <= 3.0):
        errors.append("Радиальный припуск наружной кромки должен быть 0…3 мм.")
    if not (4 <= b.main_blades <= 24):
        errors.append("main_blades должен быть в диапазоне 4…24.")
    if not (0.15 <= b.splitter_start_fraction <= 0.80):
        errors.append("splitter_start_fraction должен быть в диапазоне 0.15…0.80.")
    if not (0.25 <= b.splitter_pitch_fraction <= 0.75):
        errors.append("splitter_pitch_fraction должен быть в диапазоне 0.25…0.75.")
    if not (0.0 <= b.backsweep_from_radial_deg <= 60.0):
        errors.append("backsweep_from_radial_deg должен быть 0…60°.")
    if b.inlet_angle_mode not in {"auto", "manual"}:
        errors.append("inlet_angle_mode: auto или manual.")
    if b.root_embed_mm <= 0.05 or b.root_embed_mm >= 0.35 * g.b2_mm:
        errors.append("root_embed_mm должен быть >0.05 и <0.35*b2.")
    if not (0.0 <= b.main_root_le_trim_fraction <= 0.12):
        errors.append("main_root_le_trim_fraction должен быть 0…0.12.")
    if min(b.t_le_hub_mm, b.t_le_shroud_mm, b.t_te_hub_mm, b.t_te_shroud_mm) <= 0.05:
        errors.append("Толщина LE/TE должна быть >0.05 мм.")
    if not (0.15 <= b.max_thickness_location_hub <= 0.65 and
            0.15 <= b.max_thickness_location_shroud <= 0.65):
        errors.append("Положение максимальной толщины должно быть 0.15…0.65 хорды.")
    if b.chord_samples < 41 or b.span_sections < 3:
        errors.append("Слишком грубая дискретизация лопатки.")
    if o.rpm <= 0 or o.t01_k <= 0 or o.p01_pa <= 0:
        errors.append("Некорректная рабочая точка.")
    if not (0.05 <= o.flow_coefficient_phi2 <= 0.50):
        errors.append("flow_coefficient_phi2 должен быть 0.05…0.50.")
    if not (0.5 <= o.blockage_inlet <= 1.0 and 0.5 <= o.blockage_exit <= 1.0):
        errors.append("blockage должен быть 0.5…1.0.")
    if m.material not in MATERIALS:
        errors.append(f"Неизвестный материал: {m.material}")
    if cfg.output.mesh_quality not in QUALITY:
        errors.append("mesh_quality: draft / normal / fine.")
    if not (0.0 <= cfg.output.mesh_simplify_tolerance_mm <= 0.05):
        errors.append("mesh_simplify_tolerance_mm должен быть 0…0.05 мм.")
    if errors:
        raise ValueError("\n".join(errors))


# =============================================================================
#                             ВСПОМОГАТЕЛЬНОЕ
# =============================================================================


def smootherstep(x: np.ndarray) -> np.ndarray:
    return x**3 * (x * (x * 6.0 - 15.0) + 10.0)


def bezier4(points: list[tuple[float, float]], u: np.ndarray) -> np.ndarray:
    uu = u[:, None]
    p0, p1, p2, p3 = [np.asarray(p, dtype=float) for p in points]
    return (
        (1.0 - uu) ** 3 * p0
        + 3.0 * (1.0 - uu) ** 2 * uu * p1
        + 3.0 * (1.0 - uu) * uu**2 * p2
        + uu**3 * p3
    )


def normalized_rows(values: np.ndarray) -> np.ndarray:
    return values / np.maximum(np.linalg.norm(values, axis=-1, keepdims=True), 1e-12)


def continuous_rows(values: np.ndarray) -> np.ndarray:
    out = values.copy()
    for i in range(1, len(out)):
        if float(np.dot(out[i], out[i - 1])) < 0.0:
            out[i] *= -1.0
    return out


def trapezoid_integral(y: np.ndarray, x: np.ndarray) -> float:
    """Совместимость с NumPy 1.x (`trapz`) и NumPy 2.x (`trapezoid`)."""
    function = getattr(np, "trapezoid", None)
    if function is None:  # pragma: no cover - для старых NumPy
        function = np.trapz
    return float(function(y, x))


def rotate_mesh_z(mesh: trimesh.Trimesh, angle_rad: float) -> trimesh.Trimesh:
    result = mesh.copy()
    result.apply_transform(trimesh.transformations.rotation_matrix(angle_rad, [0, 0, 1]))
    return result


def clean_mesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    result = mesh.copy()
    result.merge_vertices(digits_vertex=8)
    try:
        result.update_faces(result.nondegenerate_faces(height=1e-12))
    except TypeError:
        result.update_faces(result.nondegenerate_faces())
    try:
        result.update_faces(result.unique_faces())
    except AttributeError:  # pragma: no cover - старые trimesh
        pass
    result.remove_unreferenced_vertices()
    # process(validate=True) в новых trimesh неявно требует optional networkx.
    # Здесь ориентация граней уже задаётся генератором/Manifold; ниже она явно
    # проверяется, поэтому скрытая ремонтная операция не нужна.
    if result.is_watertight and not result.is_winding_consistent:
        result = orient_mesh_faces(result)
    if result.volume < 0.0:
        result.invert()
    return result


def orient_mesh_faces(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Согласует winding соседних треугольников без optional networkx."""
    result = mesh.copy()
    faces = np.asarray(result.faces, dtype=np.int64).copy()
    adjacency = np.asarray(result.face_adjacency, dtype=np.int64)
    shared_edges = np.asarray(result.face_adjacency_edges, dtype=np.int64)
    neighbours: list[list[tuple[int, int, int]]] = [[] for _ in range(len(faces))]

    def edge_direction(face: np.ndarray, u: int, v: int) -> int:
        for k in range(3):
            if int(face[k]) == u and int(face[(k + 1) % 3]) == v:
                return 1
            if int(face[k]) == v and int(face[(k + 1) % 3]) == u:
                return -1
        raise RuntimeError("Общее ребро не найдено в треугольнике.")

    for (fa, fb), (u, v) in zip(adjacency, shared_edges):
        da = edge_direction(faces[fa], int(u), int(v))
        db = edge_direction(faces[fb], int(u), int(v))
        neighbours[int(fa)].append((int(fb), da, db))
        neighbours[int(fb)].append((int(fa), db, da))

    # state=+1 сохраняет face, -1 меняет местами вторую/третью вершины.
    state = np.zeros(len(faces), dtype=np.int8)
    for start in range(len(faces)):
        if state[start] != 0:
            continue
        state[start] = 1
        stack = [start]
        while stack:
            current = stack.pop()
            for other, d_current, d_other in neighbours[current]:
                required = int(-d_current * int(state[current]) * d_other)
                if state[other] == 0:
                    state[other] = required
                    stack.append(other)
                elif state[other] != required:
                    raise RuntimeError("Mesh неориентируем или имеет не-manifold ребро.")
    flip = np.flatnonzero(state < 0)
    faces[flip] = faces[flip][:, [0, 2, 1]]
    result.faces = faces
    return result


def mesh_body_count(mesh: trimesh.Trimesh) -> int:
    """Число связных тел без необъявленной optional-зависимости networkx."""
    faces = np.asarray(mesh.faces, dtype=np.int64)
    if len(faces) == 0:
        return 0
    parent = np.arange(len(mesh.vertices), dtype=np.int64)

    def find(i: int) -> int:
        root = i
        while parent[root] != root:
            root = int(parent[root])
        while parent[i] != i:
            nxt = int(parent[i])
            parent[i] = root
            i = nxt
        return root

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for a, b, c in faces:
        union(int(a), int(b))
        union(int(a), int(c))
    referenced = np.unique(faces)
    return len({find(int(i)) for i in referenced})



def simplify_manifold_mesh(mesh: trimesh.Trimesh, tolerance_mm: float) -> trimesh.Trimesh:
    """Удаляет sliver-треугольники в пределах строгого геометрического допуска."""
    if tolerance_mm <= 0.0:
        return clean_mesh(mesh)
    source = manifold3d.Mesh(
        np.asarray(mesh.vertices, dtype=np.float32),
        np.asarray(mesh.faces, dtype=np.uint32),
    )
    manifold = manifold3d.Manifold(source)
    if manifold.status() != manifold3d.Error.NoError:
        raise RuntimeError(f"Manifold simplify: входной mesh невалиден: {manifold.status()}")
    simplified = manifold.simplify(float(tolerance_mm)).to_mesh()
    result = trimesh.Trimesh(
        np.asarray(simplified.vert_properties[:, :3], dtype=float),
        np.asarray(simplified.tri_verts, dtype=np.int64),
        process=False,
    )
    return clean_mesh(result)


# =============================================================================
#                         ПРЕДВАРИТЕЛЬНАЯ АЭРОДИНАМИКА
# =============================================================================


class AerodynamicCalculator:
    """Предварительный mean-line расчёт с геометрической проверкой blockage.

    Коэффициенты blockage_inlet/exit трактуются как эмпирические полные
    коэффициенты открытой площади. Площадь не может быть больше геометрически
    доступной после учёта конечной толщины лопаток.
    """

    def __init__(self, cfg: DesignConfig):
        self.cfg = cfg

    def _leading_edge_open_fraction(self, c1m: float, omega: float) -> float:
        o, g, b = self.cfg.operating, self.cfg.geometry, self.cfg.blades
        r_h = g.D1_hub_mm / 2000.0
        r_s = g.D1_shroud_mm / 2000.0
        r = np.linspace(r_h, r_s, 241)
        v = (r - r_h) / max(r_s - r_h, 1e-12)
        thickness = ((1.0 - v) * b.t_le_hub_mm + v * b.t_le_shroud_mm) / 1000.0
        U = omega * r
        cu1 = o.prewhirl_ratio * U
        beta = np.arctan2(c1m, np.maximum(U - cu1, 1e-9)) + math.radians(b.incidence_deg)
        beta = np.clip(beta, math.radians(7.0), math.radians(83.0))
        projected_width = thickness / np.maximum(np.sin(beta), 1e-6)
        blocked_area = b.main_blades * trapezoid_integral(projected_width, r)
        annulus_area = math.pi * (r_s**2 - r_h**2)
        return float(np.clip(1.0 - blocked_area / max(annulus_area, 1e-12), 0.35, 1.0))

    def _trailing_edge_open_fraction(self, beta2: float) -> float:
        g, b = self.cfg.geometry, self.cfg.blades
        R2 = g.D2_mm / 2000.0
        b2 = g.b2_mm / 1000.0
        main_t = 0.5 * (b.t_te_hub_mm + b.t_te_shroud_mm) / 1000.0
        blocked = b.main_blades * b2 * main_t / max(math.sin(beta2), 1e-6)
        if b.add_splitters:
            blocked += (
                b.main_blades
                * b2
                * main_t
                * b.splitter_thickness_scale
                / max(math.sin(beta2), 1e-6)
            )
        gross = 2.0 * math.pi * R2 * b2
        return float(np.clip(1.0 - blocked / max(gross, 1e-12), 0.35, 1.0))

    def calculate(self) -> dict[str, Any]:
        o, g, b, m = self.cfg.operating, self.cfg.geometry, self.cfg.blades, self.cfg.manufacturing
        R2 = g.D2_mm / 2000.0
        R1s = g.D1_shroud_mm / 2000.0
        R1h = g.D1_hub_mm / 2000.0
        Rb = g.bore_diameter_mm / 2000.0
        b2 = g.b2_mm / 1000.0
        omega = 2.0 * math.pi * o.rpm / 60.0
        U2, U1s, U1h = omega * R2, omega * R1s, omega * R1h
        beta2 = math.radians(90.0 - b.backsweep_from_radial_deg)

        # Wiesner в американской форме. Сплиттеры учитываются как частично
        # эффективные, потому что они короче полноразмерных лопаток.
        z_eff = float(b.main_blades)
        if b.add_splitters:
            z_eff += b.splitter_slip_effectiveness * b.main_blades
        sigma_main_only = 1.0 - math.sqrt(max(math.sin(beta2), 1e-9)) / max(float(b.main_blades), 1.0) ** 0.7
        sigma_base = 1.0 - math.sqrt(max(math.sin(beta2), 1e-9)) / max(z_eff, 1.0) ** 0.7
        eps_limit = math.exp(-8.16 * math.sin(beta2) / max(z_eff, 1.0))
        radius_ratio = R1s / R2
        correction = 1.0
        if radius_ratio > eps_limit and eps_limit < 0.999999:
            correction = 1.0 - ((radius_ratio - eps_limit) / (1.0 - eps_limit)) ** 3
            correction = min(1.0, max(0.55, correction))
        sigma = min(0.98, max(0.35, sigma_base * correction))

        # Screening-модель КПД нужна прежде всего для сравнительной
        # оптимизации. Ранее eta_tt была одной константой, поэтому программа
        # не могла отличить эффективную геометрию от варианта, который лишь
        # формально максимизирует Euler head. Модель калибрована так, чтобы
        # исходный вариант оставался около заданного пользователем eta_tt.
        # Это не замена RANS CFD и не заявляемая характеристика изделия.
        exit_open_geometric = self._trailing_edge_open_fraction(beta2)
        efficiency_losses = {
            "off_design_flow": 0.10 * ((o.flow_coefficient_phi2 - 0.22) / 0.18) ** 2,
            "backsweep": 0.030 * ((b.backsweep_from_radial_deg - 30.0) / 30.0) ** 2,
            "splitter_onset": (
                0.020 * ((b.splitter_start_fraction - 0.35) / 0.25) ** 2
                if b.add_splitters else 0.018
            ),
            "slip": 0.060 * (1.0 - sigma),
            "excess_blade_surface": 0.0010 * max(0.0, float(b.main_blades - 10)) ** 2,
            "exit_blockage": 0.12 * max(0.0, 0.90 - exit_open_geometric),
        }
        eta_peak_calibrated = min(0.86, o.eta_tt + 0.030)
        eta_screen = float(np.clip(
            eta_peak_calibrated - sum(efficiency_losses.values()), 0.45, 0.86
        ))

        cm2 = o.flow_coefficient_phi2 * U2
        cu2_ideal = U2 - cm2 / max(math.tan(beta2), 1e-8)
        cu2 = sigma * U2 - cm2 / max(math.tan(beta2), 1e-8)
        cu2 = max(cu2, 0.05 * U2)
        c2 = math.hypot(cm2, cu2)
        dh0 = U2 * cu2
        T02 = o.t01_k + dh0 / CP_AIR
        pressure_ratio = (1.0 + eta_screen * dh0 / (CP_AIR * o.t01_k)) ** (
            GAMMA_AIR / (GAMMA_AIR - 1.0)
        )
        p02 = o.p01_pa * pressure_ratio
        T2 = max(20.0, T02 - c2**2 / (2.0 * CP_AIR))
        p2 = p02 * (T2 / T02) ** (GAMMA_AIR / (GAMMA_AIR - 1.0))
        rho2 = p2 / (R_AIR * T2)

        exit_open_effective = min(o.blockage_exit, exit_open_geometric)
        area2_gross = 2.0 * math.pi * R2 * b2
        area2_effective = area2_gross * exit_open_effective
        mdot = rho2 * cm2 * area2_effective

        area1_gross = math.pi * (R1s**2 - R1h**2)
        rho1 = o.p01_pa / (R_AIR * o.t01_k)
        c1m = mdot / max(rho1 * area1_gross * o.blockage_inlet, 1e-12)
        T1 = o.t01_k
        inlet_open_geometric = 1.0
        inlet_open_effective = o.blockage_inlet
        for _ in range(30):
            inlet_open_geometric = self._leading_edge_open_fraction(c1m, omega)
            inlet_open_effective = min(o.blockage_inlet, inlet_open_geometric)
            c1m = mdot / max(rho1 * area1_gross * inlet_open_effective, 1e-12)
            T1 = max(30.0, o.t01_k - c1m**2 / (2.0 * CP_AIR))
            p1 = o.p01_pa * (T1 / o.t01_k) ** (GAMMA_AIR / (GAMMA_AIR - 1.0))
            rho1_new = p1 / (R_AIR * T1)
            if abs(rho1_new - rho1) < 1e-10:
                rho1 = rho1_new
                break
            rho1 = 0.5 * (rho1 + rho1_new)

        def inlet_angle(U: float) -> float:
            cu1 = o.prewhirl_ratio * U
            return math.degrees(math.atan2(c1m, max(U - cu1, 1e-9)))

        beta1_hub_flow = inlet_angle(U1h)
        beta1_shroud_flow = inlet_angle(U1s)
        if b.inlet_angle_mode == "auto":
            beta1_hub = beta1_hub_flow + b.incidence_deg
            beta1_shroud = beta1_shroud_flow + b.incidence_deg
        else:
            beta1_hub = b.manual_beta1_hub_deg
            beta1_shroud = b.manual_beta1_shroud_deg
        beta1_hub = float(np.clip(beta1_hub, 8.0, 78.0))
        beta1_shroud = float(np.clip(beta1_shroud, 8.0, 78.0))

        a1 = math.sqrt(GAMMA_AIR * R_AIR * T1)
        w1h = math.hypot(c1m, U1h * (1.0 - o.prewhirl_ratio))
        w1s = math.hypot(c1m, U1s * (1.0 - o.prewhirl_ratio))

        material = MATERIALS[m.material]
        nu, rho_m = material["nu"], material["density"]
        sigma_disk = (
            rho_m
            * omega**2
            / 4.0
            * ((3.0 + nu) * R2**2 + (1.0 - nu) * Rb**2)
        )
        sigma_blade = (
            0.5
            * rho_m
            * omega**2
            * (R2**2 - R1h**2)
            * m.stress_concentration_factor
        )
        allowable = material["yield_mpa"] * 1e6 * material["k"]
        sigma_max = max(sigma_disk, sigma_blade)
        sf = allowable / max(sigma_max, 1e-9)
        rpm_limit = o.rpm * math.sqrt(
            allowable / max(m.required_safety_factor * sigma_max, 1e-9)
        )

        warnings: list[str] = []
        if U2 / math.sqrt(GAMMA_AIR * R_AIR * o.t01_k) > 0.90:
            warnings.append("Tip Mach > 0.90: нужен трансзвуковой CFD и shock-control профилирование.")
        if w1s / a1 > 0.90:
            warnings.append("Относительное число Маха на shroud-входе > 0.90.")
        if sf < m.required_safety_factor:
            warnings.append("Screening safety factor ниже требуемого; обороты/материал нужно пересмотреть.")
        if b.add_splitters:
            warnings.append(
                "Влияние сплиттеров на slip factor задано эмпирическим коэффициентом; "
                "для финального напора требуется CFD/эксперимент."
            )
        if "PA12-CF10" in m.material:
            warnings.append(
                "PA12-CF10 анизотропен: прочностная база взята по консервативному wet-Z "
                "пределу прочности, но выпуск ротора требует испытаний печатных купонов и FEA."
            )

        return {
            "omega_rad_s": omega,
            "U2_m_s": U2,
            "U1_shroud_m_s": U1s,
            "U1_hub_m_s": U1h,
            "tip_mach": U2 / math.sqrt(GAMMA_AIR * R_AIR * o.t01_k),
            "effective_blade_count_for_slip": z_eff,
            "slip_factor_wiesner_main_only": sigma_main_only,
            "slip_factor_wiesner": sigma,
            "slip_radius_correction": correction,
            "cm2_m_s": cm2,
            "cu2_ideal_m_s": cu2_ideal,
            "cu2_m_s": cu2,
            "c2_m_s": c2,
            "beta2_metal_from_tangent_deg": math.degrees(beta2),
            "beta2_flow_from_tangent_deg": math.degrees(math.atan2(cm2, max(U2 - cu2, 1e-9))),
            "specific_work_j_kg": dh0,
            "pressure_ratio_total_to_total": pressure_ratio,
            "eta_tt_reference_input": o.eta_tt,
            "eta_tt_screening": eta_screen,
            "eta_screening_loss_terms": efficiency_losses,
            "eta_screening_note": (
                "Сравнительная mean-line loss model для оптимизатора; "
                "окончательный КПД определяется CFD/испытанием."
            ),
            "total_temperature_rise_k": T02 - o.t01_k,
            "mass_flow_kg_s": mdot,
            "inlet_volume_flow_m3_s": mdot / rho1,
            "c1_meridional_m_s": c1m,
            "beta1_hub_flow_deg": beta1_hub_flow,
            "beta1_shroud_flow_deg": beta1_shroud_flow,
            "beta1_hub_metal_deg": beta1_hub,
            "beta1_shroud_metal_deg": beta1_shroud,
            "relative_mach_hub": w1h / a1,
            "relative_mach_shroud": w1s / a1,
            "geometric_open_fraction_inlet": inlet_open_geometric,
            "effective_open_fraction_inlet": inlet_open_effective,
            "geometric_open_fraction_exit": exit_open_geometric,
            "effective_open_fraction_exit": exit_open_effective,
            "aerodynamic_power_w": mdot * dh0,
            "aerodynamic_torque_nm": mdot * dh0 / omega,
            "screen_disk_stress_mpa": sigma_disk / 1e6,
            "screen_blade_stress_mpa": sigma_blade / 1e6,
            "screen_allowable_mpa": allowable / 1e6,
            "screen_strength_basis": material.get("strength_basis", "screening strength"),
            "screen_safety_factor": sf,
            "screen_rpm_limit_at_required_sf": rpm_limit,
            "warnings": warnings,
            "screening_note": (
                "Прочность: аналитическая оценка кольцевого диска постоянной толщины "
                "и радиальной полосы лопатки; это не FEA."
            ),
        }


# =============================================================================
#                    МЕРИДИОНАЛЬНЫЙ КАНАЛ И CAMBER
# =============================================================================


class MeridionalProfiler:
    def __init__(self, cfg: DesignConfig):
        self.cfg = cfg

    def control_points(self) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
        g = self.cfg.geometry
        # Геометрия-заготовка продолжается за чистовой D2 на величину
        # припуска. И hub, и shroud получают один общий конечный радиус,
        # поэтому выходные кромки всех лопаток также доходят до Ø заготовки.
        # Аэродинамический расчёт использует отдельно чистовой g.D2_mm.
        R2 = g.D2_mm / 2.0 + g.outer_edge_finish_allowance_radial_mm
        R1h, R1s = g.D1_hub_mm / 2.0, g.D1_shroud_mm / 2.0
        L, b2 = g.axial_length_mm, g.b2_mm
        dh, ds = R2 - R1h, R2 - R1s
        hub = [
            (R1h, L),
            (R1h + g.hub_p1_dr_fraction * dh, L - g.hub_p1_drop_fraction * L),
            (R2 - g.hub_p2_dr_fraction * dh, g.hub_p2_z_fraction * L),
            (R2, 0.0),
        ]
        shroud = [
            (R1s, L),
            (R1s + g.shroud_p1_dr_fraction * ds, L - g.shroud_p1_drop_fraction * (L - b2)),
            (R2 - g.shroud_p2_dr_fraction * ds, b2 + g.shroud_p2_rise_fraction * (L - b2)),
            (R2, b2),
        ]
        return hub, shroud

    def sample(self, count: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        u = np.linspace(0.0, 1.0, int(count))
        hub_cp, shroud_cp = self.control_points()
        hub, shroud = bezier4(hub_cp, u), bezier4(shroud_cp, u)
        mid = 0.5 * (hub + shroud)
        dm = np.r_[0.0, np.hypot(np.diff(mid[:, 0]), np.diff(mid[:, 1]))]
        s = np.cumsum(dm)
        s /= max(s[-1], 1e-12)
        if np.min(np.linalg.norm(shroud - hub, axis=1)) <= 0.25:
            raise ValueError("Hub/shroud пересекаются или почти совпадают.")
        if np.any(np.diff(hub[:, 0]) < -1e-7) or np.any(np.diff(shroud[:, 0]) < -1e-7):
            raise ValueError("Радиус меридиональных линий должен монотонно расти.")
        return s, hub, shroud


@dataclass
class CamberCurve:
    s: np.ndarray
    xyz: np.ndarray
    beta_rad: np.ndarray
    theta_rad: np.ndarray
    r_mm: np.ndarray
    z_mm: np.ndarray


class CamberGenerator:
    def __init__(self, cfg: DesignConfig, aero: dict[str, Any], count: int):
        self.cfg, self.aero = cfg, aero
        self.s_full, self.hub_full, self.shroud_full = MeridionalProfiler(cfg).sample(count)

    def angle_law(self, s_local: np.ndarray, beta1: float, beta2: float) -> np.ndarray:
        # Quintic smootherstep даёт нулевой градиент угла на LE и TE.
        q = smootherstep(s_local)
        bias = self.cfg.blades.loading_bias
        if abs(bias) > 1e-12:
            q = q + bias * 64.0 * s_local**3 * (1.0 - s_local) ** 3 * (s_local - 0.5)
            q = np.maximum.accumulate(np.clip(q, 0.0, 1.0))
            q = (q - q[0]) / max(q[-1] - q[0], 1e-12)
        return beta1 + (beta2 - beta1) * q

    def _beta1_for_span(self, v: float, r_le_mm: float) -> float:
        b, o = self.cfg.blades, self.cfg.operating
        if b.inlet_angle_mode == "manual":
            vv = float(np.clip(v, 0.0, 1.0))
            return math.radians((1.0 - vv) * b.manual_beta1_hub_deg + vv * b.manual_beta1_shroud_deg)
        # В auto-режиме угол вычисляется непосредственно по локальному радиусу,
        # а не линейной интерполяцией hub/shroud углов.
        r_m = max(r_le_mm, self.cfg.geometry.D1_hub_mm / 2.0) / 1000.0
        U = self.aero["omega_rad_s"] * r_m
        cu1 = o.prewhirl_ratio * U
        beta = math.atan2(self.aero["c1_meridional_m_s"], max(U - cu1, 1e-9))
        beta += math.radians(b.incidence_deg)
        return float(np.clip(beta, math.radians(8.0), math.radians(78.0)))

    def curve(self, v: float, start_fraction: float = 0.0, phase_rad: float = 0.0) -> CamberCurve:
        b = self.cfg.blades
        s_common, hub, shroud = self.s_full, self.hub_full, self.shroud_full
        r = (1.0 - v) * hub[:, 0] + v * shroud[:, 0]
        z = (1.0 - v) * hub[:, 1] + v * shroud[:, 1]
        dm = np.r_[0.0, np.hypot(np.diff(r), np.diff(z))]
        m = np.cumsum(dm)
        s_local = m / max(m[-1], 1e-12)
        beta1 = self._beta1_for_span(v, float(r[0]))
        beta2 = math.radians(90.0 - b.backsweep_from_radial_deg)
        beta = self.angle_law(s_local, beta1, beta2)
        theta = np.zeros_like(s_common)
        for i in range(1, len(s_common)):
            rmid = max(0.5 * (r[i] + r[i - 1]), 1e-8)
            bmid = 0.5 * (beta[i] + beta[i - 1])
            theta[i] = theta[i - 1] + dm[i] / (rmid * max(math.tan(bmid), 1e-8))
        theta = (theta - theta[-1]) * b.wrap_scale
        span = np.linalg.norm(shroud - hub, axis=1)
        q = smootherstep(s_common)
        lean = np.radians(b.lean_le_deg * (1.0 - q) + b.lean_te_deg * q)
        theta += v * span * np.tan(lean) / np.maximum(r, 1e-8)
        theta += phase_rad
        xyz = np.column_stack((r * np.cos(theta), r * np.sin(theta), z))
        mask = s_common >= start_fraction - 1e-12
        return CamberCurve(s_common[mask], xyz[mask], beta[mask], theta[mask], r[mask], z[mask])


class ThicknessLaw:
    """C1-гладкая толщина с контролируемой позицией максимума.

    На каждой стороне от максимума используется quintic smootherstep. Это
    устраняет изломы и численный шум, а эллиптический нос строится отдельно.
    """

    def __init__(self, cfg: DesignConfig):
        self.cfg = cfg

    def values(self, s: np.ndarray, v: float, splitter: bool) -> np.ndarray:
        b = self.cfg.blades
        vv = float(np.clip(v, 0.0, 1.0))
        tle = (1.0 - vv) * b.t_le_hub_mm + vv * b.t_le_shroud_mm
        tmax = (1.0 - vv) * b.t_max_hub_mm + vv * b.t_max_shroud_mm
        tte = (1.0 - vv) * b.t_te_hub_mm + vv * b.t_te_shroud_mm
        xmax = (1.0 - vv) * b.max_thickness_location_hub + vv * b.max_thickness_location_shroud
        x = (s - s[0]) / max(s[-1] - s[0], 1e-12)
        x = np.clip(x, 0.0, 1.0)
        out = np.empty_like(x)
        left = x <= xmax
        if np.any(left):
            q = smootherstep(x[left] / max(xmax, 1e-8))
            out[left] = tle + (tmax - tle) * q
        if np.any(~left):
            q = smootherstep((x[~left] - xmax) / max(1.0 - xmax, 1e-8))
            out[~left] = tmax + (tte - tmax) * q
        if splitter:
            out *= b.splitter_thickness_scale
        if v < 0.0:
            out *= b.root_thickness_factor
        return out


# =============================================================================
#                     ЛИНЕЙЧАТАЯ ЛОПАТКА: ПРОФИЛИ И MESH
# =============================================================================


@dataclass
class BladeProfiles:
    s: np.ndarray
    root_camber: np.ndarray
    tip_camber: np.ndarray
    root_pressure: np.ndarray
    root_suction: np.ndarray
    tip_pressure: np.ndarray
    tip_suction: np.ndarray
    root_normal: np.ndarray
    tip_normal: np.ndarray
    root_thickness: np.ndarray
    tip_thickness: np.ndarray
    root_beta_rad: np.ndarray
    tip_beta_rad: np.ndarray


class BladeBuilder:
    def __init__(self, cfg: DesignConfig, aero: dict[str, Any], quality: str):
        self.cfg, self.aero = cfg, aero
        q = QUALITY[quality]
        self.chord_count = max(61, int(round(cfg.blades.chord_samples * q["chord_scale"])))
        if self.chord_count % 2 == 0:
            self.chord_count += 1
        self.span_sections = max(3, int(q["span_sections"]))
        self.le_arc = max(5, int(q["le_arc"]))
        if self.le_arc % 2 == 0:
            self.le_arc += 1
        self.camber = CamberGenerator(cfg, aero, self.chord_count)
        self.thickness = ThicknessLaw(cfg)

    def _profile_points(
        self,
        camber: np.ndarray,
        normal: np.ndarray,
        tangent: np.ndarray,
        thickness: np.ndarray,
        v: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        b = self.cfg.blades
        n_dir = normalized_rows(normal)
        # У trailing edge направление толщины плавно переводится в окружное.
        # Поэтому обрезанная кромка лежит на цилиндре R2, а не выступает за D2.
        radius = np.maximum(np.linalg.norm(camber[:, :2], axis=1), 1e-12)
        e_theta = np.column_stack((-camber[:, 1] / radius, camber[:, 0] / radius, np.zeros_like(radius)))
        for i in range(len(e_theta)):
            if float(np.dot(e_theta[i], n_dir[i])) < 0.0:
                e_theta[i] *= -1.0
        x_station = np.linspace(0.0, 1.0, len(camber))
        te_blend = smootherstep(np.clip((x_station - 0.86) / 0.14, 0.0, 1.0))
        offset_dir = normalized_rows((1.0 - te_blend[:, None]) * n_dir + te_blend[:, None] * e_theta)
        pressure = camber + offset_dir * thickness[:, None] / 2.0
        suction = camber - offset_dir * thickness[:, None] / 2.0

        # Точная tangential cut-off кромка на том же радиусе, что camber TE.
        theta_te = math.atan2(camber[-1, 1], camber[-1, 0])
        sign = 1.0 if float(np.dot(e_theta[-1], np.array([-math.sin(theta_te), math.cos(theta_te), 0.0]))) >= 0.0 else -1.0
        dtheta = sign * thickness[-1] / (2.0 * radius[-1])
        pressure[-1] = np.array([radius[-1] * math.cos(theta_te + dtheta), radius[-1] * math.sin(theta_te + dtheta), camber[-1, 2]])
        suction[-1] = np.array([radius[-1] * math.cos(theta_te - dtheta), radius[-1] * math.sin(theta_te - dtheta), camber[-1, 2]])

        vv = float(np.clip(v, 0.0, 1.0))
        ratio = (1.0 - vv) * b.leading_edge_ellipse_ratio_hub + vv * b.leading_edge_ellipse_ratio_shroud
        if v < 0.0:
            # Скрытая в ступице корневая секция нужна только для надёжного union;
            # длинный аэродинамический нос здесь не нужен и не должен заходить в bore.
            ratio = min(ratio, 1.0)
        semi_minor = thickness[0] / 2.0
        semi_major = max(semi_minor, ratio * semi_minor)
        center, n0 = camber[0], n_dir[0]
        t0 = tangent[0] / max(np.linalg.norm(tangent[0]), 1e-12)
        p_arc, s_arc = [], []
        for k in range(self.le_arc):
            fraction = k / (self.le_arc - 1)
            qp = math.pi / 2.0 * (1.0 - fraction)
            qs = math.pi / 2.0 + math.pi / 2.0 * fraction
            p_arc.append(center + n0 * semi_minor * math.cos(qp) - t0 * semi_major * math.sin(qp))
            s_arc.append(center + n0 * semi_minor * math.cos(qs) - t0 * semi_major * math.sin(qs))
        return np.vstack((np.asarray(p_arc), pressure[1:])), np.vstack((np.asarray(s_arc), suction[1:]))

    def profiles(self, start_fraction: float, phase_rad: float, splitter: bool) -> BladeProfiles:
        # Авторитетная аэродинамическая поверхность задаётся между hub (v=0)
        # и shroud (v=1). Корневое заглубление добавляется только в mesh().
        hub_curve = self.camber.curve(0.0, start_fraction, phase_rad)
        tip_curve = self.camber.curve(1.0, start_fraction, phase_rad)
        hub, tip = hub_curve.xyz, tip_curve.xyz
        if len(hub) != len(tip):
            raise RuntimeError("Hub/tip camber имеют разную дискретизацию.")
        span = tip - hub
        tangent_hub = np.gradient(hub, axis=0, edge_order=2)
        tangent_tip = np.gradient(tip, axis=0, edge_order=2)
        normal_hub = continuous_rows(normalized_rows(np.cross(tangent_hub, span)))
        normal_tip = continuous_rows(normalized_rows(np.cross(tangent_tip, span)))
        if float(np.mean(np.sum(normal_hub * normal_tip, axis=1))) < 0.0:
            normal_tip *= -1.0
        radial = np.maximum(np.linalg.norm(hub[:, :2], axis=1), 1e-12)
        e_theta = np.column_stack((-hub[:, 1] / radial, hub[:, 0] / radial, np.zeros_like(radial)))
        if float(np.median(np.sum(normal_hub * e_theta, axis=1))) < 0.0:
            normal_hub *= -1.0
            normal_tip *= -1.0
        th = self.thickness.values(hub_curve.s, 0.0, splitter)
        tt = self.thickness.values(tip_curve.s, 1.0, splitter)
        hp, hs = self._profile_points(hub, normal_hub, tangent_hub, th, 0.0)
        tp, ts = self._profile_points(tip, normal_tip, tangent_tip, tt, 1.0)
        return BladeProfiles(
            hub_curve.s,
            hub,
            tip,
            hp,
            hs,
            tp,
            ts,
            normal_hub,
            normal_tip,
            th,
            tt,
            hub_curve.beta_rad,
            tip_curve.beta_rad,
        )

    def _embedded_root_profiles(
        self,
        profiles: BladeProfiles,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Возвращает закрываемую секцию, гарантированно находящуюся в hub.

        Смещение выполняется против локального span-вектора, то есть по
        внутренней нормали меридионального канала. Это корректно и у входной
        втулки, где span почти радиальный, и у выхода, где он почти осевой.
        """
        base_camber = profiles.root_camber.copy()
        base_normal = profiles.root_normal.copy()
        base_thickness = profiles.root_thickness.copy()
        root_station = (
            (profiles.s - profiles.s[0])
            / max(profiles.s[-1] - profiles.s[0], 1.0e-12)
        )

        # Только у полноразмерной лопатки LE совпадала с резким стыком nose/hub.
        # Скрытый корень локально отводится downstream, а к 16% хорды плавно
        # возвращается к расчётной camber-линии. Tip и рабочая поверхность LE не
        # меняются. Корневая петля теперь целиком лежит на гладкой hub-поверхности.
        is_main_le = profiles.s[0] <= 1.0e-9
        if is_main_le and self.cfg.blades.main_root_le_trim_fraction > 0.0:
            blend_length = max(0.16, 3.5 * self.cfg.blades.main_root_le_trim_fraction)
            fade = 1.0 - smootherstep(np.clip(root_station / blend_length, 0.0, 1.0))
            mapped = np.clip(
                root_station + self.cfg.blades.main_root_le_trim_fraction * fade,
                0.0,
                1.0,
            )

            def remap(values: np.ndarray) -> np.ndarray:
                return np.column_stack([
                    np.interp(mapped, root_station, values[:, axis])
                    for axis in range(values.shape[1])
                ])

            base_camber = remap(base_camber)
            base_normal = normalized_rows(remap(base_normal))
            base_thickness = np.interp(mapped, root_station, base_thickness)

        span_unit = normalized_rows(profiles.tip_camber - base_camber)
        camber = base_camber - span_unit * self.cfg.blades.root_embed_mm

        # На TE span-вектор почти осевой: одного смещения по нему недостаточно,
        # и root-cap раньше оставался точно на наружном цилиндре R2. Это давало
        # совпадающие грани, нулевые B-Rep-рёбра и срыв fillet в SolidWorks.
        # Скрытая секция плавно уходит также радиально внутрь только на последних
        # 18% хорды. При наличии наружного припуска inset ограничивается так,
        # чтобы после ручной подрезки до чистового D2 в корне лопатки всё ещё
        # оставался небольшой материал, а не возникал недорез до Ø198 мм.
        station = np.linspace(0.0, 1.0, len(camber))
        te_weight = smootherstep(np.clip((station - 0.82) / 0.18, 0.0, 1.0))
        radial_inset = min(1.25, 0.75 * self.cfg.blades.root_embed_mm)
        allowance = self.cfg.geometry.outer_edge_finish_allowance_radial_mm
        if allowance > 1.0e-9:
            radial_inset = min(radial_inset, max(0.20, 0.65 * allowance))
        radius = np.maximum(np.linalg.norm(camber[:, :2], axis=1), 1e-12)
        target_radius = radius - te_weight * radial_inset
        camber[:, 0] *= target_radius / radius
        camber[:, 1] *= target_radius / radius

        tangent = np.gradient(camber, axis=0, edge_order=2)
        thickness = base_thickness * self.cfg.blades.root_thickness_factor
        pressure, suction = self._profile_points(
            camber, base_normal, tangent, thickness, -1.0
        )
        return camber, pressure, suction

    def mesh(self, start_fraction: float = 0.0, phase_rad: float = 0.0, splitter: bool = False) -> tuple[trimesh.Trimesh, BladeProfiles]:
        p = self.profiles(start_fraction, phase_rad, splitter)
        eta = np.linspace(0.0, 1.0, self.span_sections)
        aero_pressure = np.stack([(1.0 - v) * p.root_pressure + v * p.tip_pressure for v in eta], axis=1)
        aero_suction = np.stack([(1.0 - v) * p.root_suction + v * p.tip_suction for v in eta], axis=1)

        # Та же заглублённая секция используется и в B-Rep, поэтому STL и STEP
        # имеют одинаковую физическую логику корневого соединения.
        _, root_pressure, root_suction = self._embedded_root_profiles(p)
        pressure = np.concatenate((root_pressure[:, None, :], aero_pressure), axis=1)
        suction = np.concatenate((root_suction[:, None, :], aero_suction), axis=1)
        nc, nv = pressure.shape[:2]

        vertices: list[np.ndarray] = []
        ip = np.empty((nc, nv), dtype=np.int64)
        is_ = np.empty((nc, nv), dtype=np.int64)
        # Общая nose-линия: pressure и suction используют одинаковые индексы.
        for j in range(nv):
            ip[0, j] = is_[0, j] = len(vertices)
            vertices.append(pressure[0, j])
        for i in range(1, nc):
            for j in range(nv):
                ip[i, j] = len(vertices)
                vertices.append(pressure[i, j])
        for i in range(1, nc):
            for j in range(nv):
                is_[i, j] = len(vertices)
                vertices.append(suction[i, j])

        faces: list[tuple[int, int, int]] = []
        for i in range(nc - 1):
            for j in range(nv - 1):
                a, bb, c, d = ip[i, j], ip[i + 1, j], ip[i + 1, j + 1], ip[i, j + 1]
                faces.extend(((a, bb, c), (a, c, d)))
                a, bb, c, d = is_[i, j], is_[i, j + 1], is_[i + 1, j + 1], is_[i + 1, j]
                faces.extend(((a, bb, c), (a, c, d)))
        # Заглублённый root cap и свободный shroud-tip cap.
        for jspan, reverse in ((0, False), (nv - 1, True)):
            nose, p1, s1 = ip[0, jspan], ip[1, jspan], is_[1, jspan]
            faces.append((nose, p1, s1) if reverse else (nose, s1, p1))
            for i in range(1, nc - 1):
                p0, p1 = ip[i, jspan], ip[i + 1, jspan]
                s0, s1 = is_[i, jspan], is_[i + 1, jspan]
                if reverse:
                    faces.extend(((p0, p1, s1), (p0, s1, s0)))
                else:
                    faces.extend(((p0, s0, s1), (p0, s1, p1)))
        # Cut-off trailing edge.
        i = nc - 1
        for j in range(nv - 1):
            p0, p1, s0, s1 = ip[i, j], ip[i, j + 1], is_[i, j], is_[i, j + 1]
            faces.extend(((p0, p1, s1), (p0, s1, s0)))

        mesh = clean_mesh(trimesh.Trimesh(np.asarray(vertices), np.asarray(faces), process=False))
        bodies = mesh_body_count(mesh)
        if not mesh.is_watertight or not mesh.is_winding_consistent or bodies != 1:
            raise RuntimeError(
                "Сгенерированная лопатка не является герметичным телом: "
                f"watertight={mesh.is_watertight}, winding={mesh.is_winding_consistent}, "
                f"bodies={bodies}."
            )
        return mesh, p

    @staticmethod
    def _cqv(point: Iterable[float]) -> cq.Vector:
        q = list(point)
        return cq.Vector(float(q[0]), float(q[1]), float(q[2]))

    @classmethod
    def _edge(cls, points: np.ndarray) -> cq.Edge:
        return cq.Edge.makeSpline([cls._cqv(q) for q in points], tol=1e-7)

    def reference_brep(self, profiles: BladeProfiles) -> cq.Shape:
        """Гладкая STEP-лопатка с root-cap глубоко внутри ступицы."""
        _, root_pressure, root_suction = self._embedded_root_profiles(profiles)
        rp, rs = self._edge(root_pressure), self._edge(root_suction)
        tp, ts = self._edge(profiles.tip_pressure), self._edge(profiles.tip_suction)
        root_te = cq.Edge.makeLine(self._cqv(root_pressure[-1]), self._cqv(root_suction[-1]))
        tip_te = cq.Edge.makeLine(
            self._cqv(profiles.tip_pressure[-1]), self._cqv(profiles.tip_suction[-1])
        )
        # Одна непрерывная ruled-face на каждой стороне исключает искусственную
        # секционную кромку возле hub. После fuse остаётся только настоящая
        # внешняя линия сопряжения, которую SolidWorks может корректно fillet.
        faces = [
            cq.Face.makeRuledSurface(rp, tp),
            cq.Face.makeRuledSurface(rs, ts),
            cq.Face.makeRuledSurface(rp, rs),
            cq.Face.makeRuledSurface(tp, ts),
            cq.Face.makeRuledSurface(root_te, tip_te),
        ]
        shape = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
        if not shape.isValid():
            shape = shape.fix()
        if not shape.isValid() or shape.Volume() <= 0.0:
            raise RuntimeError("Не удалось построить reference B-Rep лопатки.")
        return shape


# =============================================================================
#                                  СТУПИЦА
# =============================================================================


class HubBuilder:
    def __init__(self, cfg: DesignConfig):
        self.cfg = cfg
        self.profiler = MeridionalProfiler(cfg)

    @staticmethod
    def _v(r: float, z: float) -> cq.Vector:
        return cq.Vector(float(r), 0.0, float(z))

    def brep(self) -> cq.Shape:
        g, m = self.cfg.geometry, self.cfg.manufacturing
        R2, R1h, Rb = g.D2_mm / 2.0, g.D1_hub_mm / 2.0, g.bore_diameter_mm / 2.0
        R_stock = R2 + g.outer_edge_finish_allowance_radial_mm
        L, tb = g.axial_length_mm, g.backplate_thickness_mm
        hub_cp, _ = self.profiler.control_points()
        edges = [
            cq.Edge.makeLine(self._v(Rb, -tb), self._v(R_stock, -tb)),
            cq.Edge.makeLine(self._v(R_stock, -tb), self._v(R_stock, 0.0)),
        ]
        # Без отдельной кольцевой полки: meridional Bezier уже заканчивается
        # на R_stock, поэтому наружная поверхность диска и лопатки образуют
        # единую заготовку Ø202 без лишней концентрической линии на плоскости.
        edges.extend([
            cq.Edge.makeBezier([
                self._v(*hub_cp[3]), self._v(*hub_cp[2]), self._v(*hub_cp[1]), self._v(*hub_cp[0])
            ]),
            cq.Edge.makeLine(self._v(R1h, L), self._v(Rb, L + g.nose_extra_height_mm)),
            cq.Edge.makeLine(self._v(Rb, L + g.nose_extra_height_mm), self._v(Rb, -tb)),
        ])
        hub = cq.Solid.revolve(cq.Wire.assembleEdges(edges), [], 360.0, (0, 0, 0), (0, 0, 1))
        if m.keyway_enabled:
            z0 = -tb - 1.0
            cutter = cq.Solid.makeBox(
                m.keyway_depth_mm + 0.6,
                m.keyway_width_mm,
                L + g.nose_extra_height_mm + tb + 2.0,
                cq.Vector(Rb - 0.3, -m.keyway_width_mm / 2.0, z0),
            )
            hub = hub.cut(cutter, tol=1e-3)
        if not hub.isValid():
            hub = hub.fix()
        if not hub.isValid() or hub.Volume() <= 0.0:
            raise RuntimeError("Невалидная геометрия ступицы.")
        return hub

    @staticmethod
    def to_mesh(shape: cq.Shape, linear: float, angular: float) -> trimesh.Trimesh:
        vertices, faces = shape.tessellate(linear, angular)
        mesh = trimesh.Trimesh(
            np.asarray([[q.x, q.y, q.z] for q in vertices], dtype=float),
            np.asarray(faces, dtype=np.int64),
            process=False,
        )
        mesh = clean_mesh(mesh)
        if not mesh.is_watertight:
            raise RuntimeError("Tessellation ступицы не watertight.")
        return mesh


# =============================================================================
#                       БЫСТРАЯ ПРОВЕРКА ГОРЛА
# =============================================================================


class ThroatChecker:
    def __init__(self, cfg: DesignConfig, aero: dict[str, Any]):
        self.cfg = cfg
        self.camber = CamberGenerator(cfg, aero, max(91, cfg.blades.chord_samples))
        self.thickness = ThicknessLaw(cfg)

    def _clearance(self, v: float, phase_a: float, start_a: float, split_a: bool, phase_b: float, start_b: float, split_b: bool) -> tuple[float, float]:
        a, b = self.camber.curve(v, start_a, phase_a), self.camber.curve(v, start_b, phase_b)
        start = max(start_a, start_b)
        ca, cb = a.xyz[a.s >= start - 1e-12], b.xyz[b.s >= start - 1e-12]
        sa, sb = a.s[a.s >= start - 1e-12], b.s[b.s >= start - 1e-12]
        n = min(len(ca), len(cb))
        ca, cb, sa, sb = ca[:n], cb[:n], sa[:n], sb[:n]
        ta, tb = self.thickness.values(sa, v, split_a), self.thickness.values(sb, v, split_b)
        values = np.linalg.norm(ca - cb, axis=1) - 0.5 * (ta + tb)
        idx = int(np.argmin(values))
        return float(values[idx]), float(sa[idx])

    def calculate(self) -> dict[str, Any]:
        b = self.cfg.blades
        pitch = 2.0 * math.pi / b.main_blades
        records: list[dict[str, Any]] = []
        for v in (0.05, 0.50, 0.95):
            if b.add_splitters:
                split_phase = pitch * b.splitter_pitch_fraction
                for name, args in (
                    ("main-splitter", (0.0, 0.0, False, split_phase, b.splitter_start_fraction, True)),
                    ("splitter-next-main", (split_phase, b.splitter_start_fraction, True, pitch, 0.0, False)),
                    ("main-main-before-splitter", (0.0, 0.0, False, pitch, 0.0, False)),
                ):
                    value, s = self._clearance(v, *args)
                    if name == "main-main-before-splitter":
                        # Ограничиваем область до начала сплиттера.
                        a = self.camber.curve(v, 0.0, 0.0)
                        mask = a.s <= b.splitter_start_fraction + 1e-12
                        if np.any(mask):
                            c0 = a.xyz[mask]
                            c1 = self.camber.curve(v, 0.0, pitch).xyz[mask]
                            t = self.thickness.values(a.s[mask], v, False)
                            vv = np.linalg.norm(c0 - c1, axis=1) - t
                            ii = int(np.argmin(vv)); value, s = float(vv[ii]), float(a.s[mask][ii])
                    records.append(dict(pair=name, span=v, s=s, clearance_mm=value))
            else:
                value, s = self._clearance(v, 0.0, 0.0, False, pitch, 0.0, False)
                records.append(dict(pair="main-main", span=v, s=s, clearance_mm=value))
        worst = min(records, key=lambda x: x["clearance_mm"])
        return {
            "minimum_approximate_clearance_mm": worst["clearance_mm"],
            "worst_pair": worst["pair"],
            "worst_span_fraction": worst["span"],
            "worst_meridional_fraction": worst["s"],
            "records": records,
            "note": (
                "Быстрая проверка сравнивает соседние camber-линии на одинаковой "
                "меридиональной станции и вычитает толщины; это не точная 3D throat-area оптимизация."
            ),
        }


# =============================================================================
#                              ЭКСПОРТ И ОТЧЁТ
# =============================================================================


class Exporter:
    def __init__(self, cfg: DesignConfig, out_dir: Path):
        self.cfg, self.out_dir = cfg, out_dir
        self.step_validation: dict[str, Any] | None = None
        self.out_dir.mkdir(parents=True, exist_ok=True)

    def base_name(self) -> str:
        g, b = self.cfg.geometry, self.cfg.blades
        suffix = f"Z{b.main_blades}x2" if b.add_splitters else f"Z{b.main_blades}"
        return f"impeller_D{int(round(g.D2_mm))}_{suffix}"

    @staticmethod
    def _checked_common_volume(hub: cq.Shape, blade: cq.Shape, name: str) -> float:
        common = hub.intersect(blade)
        volume = float(common.Volume())
        if not common.isValid() or volume <= MIN_ROOT_COMMON_VOLUME_MM3:
            raise RuntimeError(
                f"{name}: корень не имеет надёжного объёмного пересечения с hub "
                f"(common={volume:.6g} мм³)."
            )
        return volume

    def _fused_impeller_brep(
        self,
        hub: cq.Shape,
        main: cq.Shape,
        splitter: cq.Shape | None,
    ) -> cq.Shape:
        """Булево объединяет hub и все лопатки в единственный CAD-solid."""
        pitch_deg = 360.0 / self.cfg.blades.main_blades
        main_common = self._checked_common_volume(hub, main, "main_blade")
        splitter_common = None
        if splitter is not None:
            splitter_common = self._checked_common_volume(hub, splitter, "splitter_blade")

        blades: list[cq.Shape] = []
        for k in range(self.cfg.blades.main_blades):
            angle = k * pitch_deg
            blades.append(main.rotate((0, 0, 0), (0, 0, 1), angle))
            if splitter is not None:
                blades.append(splitter.rotate((0, 0, 0), (0, 0, 1), angle))

        # Fuzzy tolerance здесь намеренно не задаётся: на криволинейных тонких
        # корнях OpenCascade может при ненулевом tol ошибочно съесть объём hub.
        fused = hub.fuse(*blades)
        # clean() удаляет лишние same-domain splitter-рёбра. Настоящие внешние
        # root-рёбра остаются и нормально выбираются инструментом Fillet.
        try:
            cleaned = fused.clean()
            if cleaned.isValid() and len(cleaned.Solids()) == 1:
                fused = cleaned
        except Exception:
            pass
        if not fused.isValid():
            fused = fused.fix()
        solids = fused.Solids()
        if not fused.isValid() or len(solids) != 1 or fused.Volume() <= 0.0:
            raise RuntimeError(
                f"STEP boolean не дал одно валидное тело: solids={len(solids)}, "
                f"valid={fused.isValid()}."
            )
        self.step_validation = {
            "valid": True,
            "solid_count": 1,
            "volume_mm3": float(fused.Volume()),
            "main_root_common_volume_mm3": main_common,
            "splitter_root_common_volume_mm3": splitter_common,
            "boolean_tolerance_mode": "OpenCascade default (no fuzzy tolerance)",
        }
        return fused

    def step_reference(self, hub: cq.Shape, main: cq.Shape, splitter: cq.Shape | None) -> dict[str, str]:
        files: dict[str, str] = {}
        base = self.base_name()
        if self.cfg.output.export_component_steps:
            for name, shape in (("hub", hub), ("main_blade", main), ("splitter_blade", splitter)):
                if shape is None:
                    continue
                path = self.out_dir / f"{base}_{name}.step"
                cq.exporters.export(cq.Workplane(obj=shape), str(path))
                files[name] = str(path)
        if self.cfg.output.export_step_reference_assembly:
            path_fused = self.out_dir / f"{base}_fused.step"
            fused = self._fused_impeller_brep(hub, main, splitter)
            cq.exporters.export(cq.Workplane(obj=fused), str(path_fused))
            files["step_fused"] = str(path_fused)
        return files

    def csv_profiles(self, profiles: BladeProfiles) -> str:
        path = self.out_dir / f"{self.base_name()}_blade_sections.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                "s", "root_x_mm", "root_y_mm", "root_z_mm", "tip_x_mm", "tip_y_mm", "tip_z_mm",
                "root_beta_deg", "tip_beta_deg", "root_thickness_mm", "tip_thickness_mm"
            ])
            # Profile arrays include LE arc, camber/thickness arrays do not; CSV uses camber stations.
            for i in range(len(profiles.s)):
                w.writerow([
                    profiles.s[i], *profiles.root_camber[i], *profiles.tip_camber[i],
                    math.degrees(profiles.root_beta_rad[i]), math.degrees(profiles.tip_beta_rad[i]),
                    profiles.root_thickness[i], profiles.tip_thickness[i],
                ])
        return str(path)


def validate_final_mesh(mesh: trimesh.Trimesh) -> dict[str, Any]:
    areas = mesh.area_faces
    radial = np.linalg.norm(np.asarray(mesh.vertices)[:, :2], axis=1)
    return {
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "body_count": mesh_body_count(mesh),
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.faces)),
        "euler_number": int(mesh.euler_number),
        "volume_mm3": float(mesh.volume),
        "bounds_mm": np.asarray(mesh.bounds).tolist(),
        "minimum_radius_mm": float(np.min(radial)),
        "maximum_radius_mm": float(np.max(radial)),
        "maximum_outer_diameter_mm": float(2.0 * np.max(radial)),
        "minimum_triangle_area_mm2": float(np.min(areas)) if len(areas) else None,
        "degenerate_triangle_count": int(np.count_nonzero(areas < 1e-12)),
    }


def print_summary(cfg: DesignConfig, aero: dict[str, Any], throat: dict[str, Any]) -> None:
    b = cfg.blades
    splitter_text = f" + {b.main_blades} splitters" if b.add_splitters else ""
    print("\n" + "═" * 76)
    print("  IMPELLER GENERATOR — расчётный baseline")
    print("═" * 76)
    print(f"  D2 = {cfg.geometry.D2_mm:g} мм | Z = {b.main_blades}{splitter_text} | N = {cfg.operating.rpm:,.0f} об/мин")
    if cfg.geometry.outer_edge_finish_allowance_radial_mm > 0.0:
        stock_d = cfg.geometry.D2_mm + 2.0 * cfg.geometry.outer_edge_finish_allowance_radial_mm
        print(
            f"  Общая заготовка диска и лопаток Ø{stock_d:.2f} мм → "
            f"совместная подрезка до Ø{cfg.geometry.D2_mm:.2f} мм"
        )
    print(f"  U2 = {aero['U2_m_s']:.1f} м/с | π_tt ≈ {aero['pressure_ratio_total_to_total']:.3f} | ṁ ≈ {aero['mass_flow_kg_s']:.4f} кг/с")
    print(f"  beta1 hub/shroud = {aero['beta1_hub_metal_deg']:.1f}° / {aero['beta1_shroud_metal_deg']:.1f}° | beta2 = {aero['beta2_metal_from_tangent_deg']:.1f}°")
    print(f"  open area inlet/exit = {aero['effective_open_fraction_inlet']:.3f} / {aero['effective_open_fraction_exit']:.3f}")
    print(f"  screening SF = {aero['screen_safety_factor']:.2f} | approx. min channel clearance = {throat['minimum_approximate_clearance_mm']:.3f} мм")


def generate(cfg: DesignConfig, output_override: str | None = None) -> dict[str, Any]:
    require_cad_dependencies()
    validate_config(cfg)
    aero = AerodynamicCalculator(cfg).calculate()
    throat = ThroatChecker(cfg, aero).calculate()
    print_summary(cfg, aero, throat)
    out_dir = Path(output_override or cfg.output.output_directory).expanduser().resolve()
    exporter = Exporter(cfg, out_dir)
    quality = QUALITY[cfg.output.mesh_quality]
    warnings = list(aero["warnings"])

    print("\n[1/7] Построение hub и параметрических лопаток...")
    t0 = time.time()
    hub_brep = HubBuilder(cfg).brep()
    blade_builder = BladeBuilder(cfg, aero, cfg.output.mesh_quality)
    main_mesh, main_profiles = blade_builder.mesh()
    split_mesh: trimesh.Trimesh | None = None
    split_profiles: BladeProfiles | None = None
    splitter_phase = 2.0 * math.pi / cfg.blades.main_blades * cfg.blades.splitter_pitch_fraction
    if cfg.blades.add_splitters:
        split_mesh, split_profiles = blade_builder.mesh(cfg.blades.splitter_start_fraction, splitter_phase, True)
    print(f"      компоненты готовы за {time.time() - t0:.2f} с.")

    files: dict[str, str] = {}
    main_brep = splitter_brep = None
    if cfg.output.export_step_reference_assembly or cfg.output.export_component_steps:
        print("[2/7] Построение и boolean-union единого STEP solid...")
        main_brep = blade_builder.reference_brep(main_profiles)
        if split_profiles is not None:
            splitter_brep = blade_builder.reference_brep(split_profiles)
        files.update(exporter.step_reference(hub_brep, main_brep, splitter_brep))
        if exporter.step_validation is not None:
            print(
                "      STEP: valid=True, solids=1, "
                f"root common(main)={exporter.step_validation['main_root_common_volume_mm3']:.3f} мм³."
            )
    else:
        print("[2/7] STEP отключён конфигурацией.")

    print("[3/7] Tessellation ступицы...")
    hub_mesh = HubBuilder.to_mesh(hub_brep, quality["hub_linear"], quality["hub_angular"])

    print("[4/7] Проверка шаблонов и Manifold boolean union...")
    meshes: list[trimesh.Trimesh] = [hub_mesh]
    pitch = 2.0 * math.pi / cfg.blades.main_blades
    for k in range(cfg.blades.main_blades):
        meshes.append(rotate_mesh_z(main_mesh, k * pitch))
        if split_mesh is not None:
            meshes.append(rotate_mesh_z(split_mesh, k * pitch))

    # Гладкая визуальная сборка без boolean-триангуляции в корневых стыках.
    # Она предназначена только для просмотра; производственный объём ниже
    # обязательно объединяется в одно manifold-тело.
    if cfg.output.export_smooth_visual_glb:
        visual = trimesh.util.concatenate(meshes)
        _ = visual.vertex_normals
        visual_path = out_dir / f"{exporter.base_name()}_smooth_visual_assembly.glb"
        visual.export(visual_path)
        files["smooth_visual_glb"] = str(visual_path)

    t_union = time.time()
    final_mesh = trimesh.boolean.union(meshes, engine="manifold", check_volume=True)
    if isinstance(final_mesh, list):
        final_mesh = trimesh.util.concatenate(final_mesh)
    final_mesh = clean_mesh(final_mesh)
    final_mesh = simplify_manifold_mesh(final_mesh, cfg.output.mesh_simplify_tolerance_mm)
    if cfg.manufacturing.scale_xy != 1.0 or cfg.manufacturing.scale_z != 1.0:
        final_mesh.apply_transform(np.diag([
            cfg.manufacturing.scale_xy,
            cfg.manufacturing.scale_xy,
            cfg.manufacturing.scale_z,
            1.0,
        ]))
        final_mesh = clean_mesh(final_mesh)
    validation = validate_final_mesh(final_mesh)
    # Геометрический контроль выполняется против реально экспортируемого
    # shrink-compensated размера, а номиналы сохраняются отдельно.
    validation["nominal_D2_mm"] = cfg.geometry.D2_mm
    validation["finished_outer_diameter_mm"] = cfg.geometry.D2_mm
    validation["outer_edge_finish_allowance_radial_mm"] = (
        cfg.geometry.outer_edge_finish_allowance_radial_mm
    )
    validation["nominal_stock_outer_diameter_mm"] = (
        cfg.geometry.D2_mm
        + 2.0 * cfg.geometry.outer_edge_finish_allowance_radial_mm
    )
    validation["target_D2_mm"] = (
        validation["nominal_stock_outer_diameter_mm"] * cfg.manufacturing.scale_xy
    )
    validation["target_stock_outer_diameter_mm"] = validation["target_D2_mm"]
    validation["outer_diameter_deviation_mm"] = (
        validation["maximum_outer_diameter_mm"] - validation["target_D2_mm"]
    )
    validation["stock_outer_diameter_deviation_mm"] = validation["outer_diameter_deviation_mm"]
    base_root_inset = min(1.25, 0.75 * cfg.blades.root_embed_mm)
    allowance = cfg.geometry.outer_edge_finish_allowance_radial_mm
    root_inset = (
        min(base_root_inset, max(0.20, 0.65 * allowance))
        if allowance > 1.0e-9 else base_root_inset
    )
    validation["embedded_root_te_radial_inset_mm"] = root_inset
    validation["approx_embedded_root_stock_beyond_finish_radius_mm"] = allowance - root_inset
    validation["finish_instruction"] = (
        f"Подрезать общий припуск диска и выходных кромок лопаток с Ø"
        f"{validation['nominal_stock_outer_diameter_mm']:.3f} мм до чистового "
        f"Ø{cfg.geometry.D2_mm:.3f} мм."
    )
    validation["nominal_bore_diameter_mm"] = cfg.geometry.bore_diameter_mm
    validation["target_bore_diameter_mm"] = (
        cfg.geometry.bore_diameter_mm * cfg.manufacturing.scale_xy
    )
    validation["bore_diameter_deviation_mm"] = (
        2.0 * validation["minimum_radius_mm"] - validation["target_bore_diameter_mm"]
    )
    print(f"      union за {time.time() - t_union:.2f} с; {validation['triangles']:,} треугольников.")
    if not validation["watertight"] or not validation["winding_consistent"] or validation["body_count"] != 1:
        raise RuntimeError(f"Финальный mesh не прошёл проверку: {validation}")
    if abs(validation["outer_diameter_deviation_mm"]) > 0.02:
        raise RuntimeError(f"Наружный диаметр вышел за допуск: {validation['outer_diameter_deviation_mm']:+.4f} мм")
    if abs(validation["bore_diameter_deviation_mm"]) > 0.02:
        raise RuntimeError(f"Диаметр bore вышел за допуск: {validation['bore_diameter_deviation_mm']:+.4f} мм")

    print("[5/7] Экспорт STL / 3MF / GLB...")
    base = exporter.base_name()
    stl_path: Path | None = None
    if cfg.output.export_stl:
        stl_path = out_dir / f"{base}_watertight.stl"
        final_mesh.export(stl_path)
        files["stl"] = str(stl_path)
    if cfg.output.export_3mf:
        path = out_dir / f"{base}_watertight.3mf"
        try:
            final_mesh.export(path)
            files["3mf"] = str(path)
        except ModuleNotFoundError as exc:
            # В некоторых версиях trimesh 3MF-плагин требует optional networkx.
            dependency = exc.name or "networkx"
            warnings.append(
                f"3MF пропущен: отсутствует {dependency}; установите `pip install networkx`."
            )
    if cfg.output.export_glb:
        path = out_dir / f"{base}_preview.glb"
        _ = final_mesh.vertex_normals
        final_mesh.export(path)
        files["glb"] = str(path)

    reload_validation = None
    if cfg.output.reload_and_validate_stl and stl_path is not None:
        reloaded = clean_mesh(trimesh.load(stl_path, force="mesh", process=False))
        reload_validation = validate_final_mesh(reloaded)
        if not reload_validation["watertight"] or reload_validation["body_count"] != 1:
            raise RuntimeError(f"STL после повторного чтения невалиден: {reload_validation}")
        print(f"      STL повторно загружен: watertight=True, bodies=1, triangles={reload_validation['triangles']:,}.")

    print("[6/7] CSV, mass properties и конфигурация...")
    if cfg.output.export_csv:
        files["blade_csv"] = exporter.csv_profiles(main_profiles)
    if cfg.output.export_metadata:
        resolved_config_path = out_dir / f"{base}_resolved_config.json"
        resolved_config_path.write_text(
            json.dumps(asdict(cfg), ensure_ascii=False, indent=2), encoding="utf-8"
        )
        files["resolved_config"] = str(resolved_config_path)

    density = MATERIALS[cfg.manufacturing.material]["density"]
    volume_mm3 = float(final_mesh.volume)
    mass_kg = volume_mm3 * density * 1e-9
    inertia_kg_m2 = np.asarray(final_mesh.moment_inertia) * density * 1e-15
    izz_kg_m2 = float(inertia_kg_m2[2, 2])
    rotational_energy_j = 0.5 * izz_kg_m2 * float(aero["omega_rad_s"]) ** 2
    angular_momentum_nms = izz_kg_m2 * float(aero["omega_rad_s"])

    if throat["minimum_approximate_clearance_mm"] < 0.50:
        warnings.append("Минимальный расчётный межлопаточный зазор <0.50 мм.")
    if cfg.manufacturing.keyway_enabled and cfg.operating.rpm > 20000:
        warnings.append("Шпоночный паз на высоких оборотах — концентратор и источник дисбаланса.")

    report = {
        "generated_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "configuration": asdict(cfg),
        "aerodynamic_screening": aero,
        "geometric_throat_screening": throat,
        "mesh_validation": {**validation, "reload_validation": reload_validation},
        "step_validation": exporter.step_validation,
        "mass_properties": {
            "material": cfg.manufacturing.material,
            "material_screening_data": MATERIALS[cfg.manufacturing.material],
            "volume_mm3": volume_mm3,
            "mass_kg": mass_kg,
            "center_of_mass_mm": np.asarray(final_mesh.center_mass).tolist(),
            "inertia_tensor_kg_m2": inertia_kg_m2.tolist(),
            "polar_inertia_izz_kg_m2": izz_kg_m2,
            "rotational_energy_j_at_design_rpm": rotational_energy_j,
            "angular_momentum_nms_at_design_rpm": angular_momentum_nms,
        },
        "warnings": warnings,
        "files": files,
        "model_status": {
            "watertight_mesh_ready_for_slicer": True,
            "step_is_fused_single_solid": bool(
                exporter.step_validation and exporter.step_validation.get("solid_count") == 1
            ),
            "smooth_visual_glb_is_fused": False,
            "step_note": (
                "STEP — единое булево-объединённое B-Rep-тело с объёмным "
                "заглублением корней лопаток в ступицу."
            ),
            "engineering_release": False,
            "release_note": (
                "Перед вращательными испытаниями нужны CFD, FEA, modal/Campbell, "
                "overspeed proof test и динамическая балансировка."
            ),
        },
    }
    if cfg.output.export_metadata:
        report_path = out_dir / f"{base}_report.json"
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        files["report"] = str(report_path)

        readme = out_dir / "README_GENERATED.txt"
        readme.write_text(
            "Финальный единый объём: *_watertight.stl и *_watertight.3mf\n"
            "GLB предназначен для гладкого визуального просмотра.\n"
            "STEP является единым fused B-Rep solid; скрытые root-cap кромки удалены boolean.\n"
            "Расчёты аэродинамики/прочности — предварительный screening, не выпуск КД.\n"
            f"Диск и лопатки имеют общий радиальный припуск "
            f"{cfg.geometry.outer_edge_finish_allowance_radial_mm:.3f} мм: "
            f"подрезать вместе до чистового Ø{cfg.geometry.D2_mm:.3f} мм.\n",
            encoding="utf-8",
        )
        files["readme"] = str(readme)

    print("[7/7] Готово.")
    print("\n" + "═" * 76)
    print("  РЕЗУЛЬТАТ")
    print("═" * 76)
    print(f"  Папка: {out_dir}")
    print(f"  Mesh: watertight=True, bodies=1, triangles={validation['triangles']:,}")
    print(f"  Объём: {volume_mm3:,.1f} мм³ | масса ({cfg.manufacturing.material}): {mass_kg * 1000:.1f} г")
    for key, value in files.items():
        print(f"  {key:26s} -> {Path(value).name}")
    if warnings:
        print("\n  Предупреждения:")
        for warning in warnings:
            print(f"   • {warning}")
    print()
    return report


# =============================================================================
#                                    CLI
# =============================================================================


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def parse_scalar(text: str) -> Any:
    low = text.strip().lower()
    if low in {"true", "yes", "on"}:
        return True
    if low in {"false", "no", "off"}:
        return False
    if low in {"none", "null"}:
        return None
    try:
        return int(text) if all(ch not in text.lower() for ch in (".", "e")) else float(text)
    except ValueError:
        return text


def set_dotted(data: dict[str, Any], dotted: str, value: Any) -> None:
    parts = dotted.split(".")
    node = data
    for part in parts[:-1]:
        if part not in node or not isinstance(node[part], dict):
            raise KeyError(f"Неизвестный путь конфигурации: {dotted}")
        node = node[part]
    if parts[-1] not in node:
        raise KeyError(f"Неизвестный параметр: {dotted}")
    node[parts[-1]] = value


# =============================================================================
#                         ТРЁХПРОФИЛЬНАЯ ОПТИМИЗАЦИЯ
# =============================================================================


OPTIMIZATION_BOUNDS: tuple[tuple[float, float], ...] = (
    (8.0, 14.0),       # main_blades, округляется до целого
    (100.0, 120.0),    # D1_shroud_mm
    (9.0, 16.0),       # b2_mm
    (0.25, 0.55),      # splitter_start_fraction
    (0.42, 0.58),      # splitter_pitch_fraction
    (5.0, 45.0),       # backsweep_from_radial_deg
    (0.15, 0.32),      # flow_coefficient_phi2
    (0.90, 1.12),      # wrap_scale
    (-0.22, 0.22),     # loading_bias
)

OPTIMIZATION_VARIABLES = (
    "main_blades", "D1_shroud_mm", "b2_mm", "splitter_start_fraction",
    "splitter_pitch_fraction", "backsweep_from_radial_deg",
    "flow_coefficient_phi2", "wrap_scale", "loading_bias",
)


def _candidate_data(base: dict[str, Any], values: Iterable[float]) -> dict[str, Any]:
    """Создаёт конфигурацию кандидата без изменения пользовательского словаря."""
    x = list(float(v) for v in values)
    data = copy.deepcopy(base)
    data["operating"]["rpm"] = float(OPTIMIZATION_PARAMETERS["design_rpm"])
    data["blades"]["main_blades"] = int(np.clip(round(x[0]), 8, 14))
    data["geometry"]["D1_shroud_mm"] = x[1]
    data["geometry"]["b2_mm"] = x[2]
    data["blades"]["splitter_start_fraction"] = x[3]
    data["blades"]["splitter_pitch_fraction"] = x[4]
    data["blades"]["backsweep_from_radial_deg"] = x[5]
    data["operating"]["flow_coefficient_phi2"] = x[6]
    data["blades"]["wrap_scale"] = x[7]
    data["blades"]["loading_bias"] = x[8]
    return data


def _flow_range_penalty(mdot: float) -> float:
    low = float(OPTIMIZATION_PARAMETERS["mass_flow_min_kg_s"])
    high = float(OPTIMIZATION_PARAMETERS["mass_flow_max_kg_s"])
    span = max(high - low, 1e-9)
    if mdot < low:
        return ((low - mdot) / span) ** 2
    if mdot > high:
        return ((mdot - high) / span) ** 2
    return 0.0


def _optimization_metrics(base: dict[str, Any], values: Iterable[float]) -> tuple[dict[str, Any], DesignConfig]:
    data = _candidate_data(base, values)
    cfg = design_from_dict(data)
    validate_config(cfg)
    aero = AerodynamicCalculator(cfg).calculate()
    pressure_rise_kpa = (aero["pressure_ratio_total_to_total"] - 1.0) * cfg.operating.p01_pa / 1000.0
    flow_penalty = _flow_range_penalty(aero["mass_flow_kg_s"])
    mach_penalty = max(0.0, aero["relative_mach_shroud"] - 0.88) ** 2 * 20.0
    open_penalty = max(0.0, 0.72 - aero["effective_open_fraction_exit"]) ** 2 * 25.0
    # wrap/loading почти не входят в mean-line напор, но чрезмерные значения
    # наказываются как риск локальной диффузии и геометрической чувствительности.
    shape_penalty = (
        0.35 * max(0.0, abs(cfg.blades.loading_bias) - 0.16) ** 2
        + 0.20 * max(0.0, abs(cfg.blades.wrap_scale - 1.0) - 0.08) ** 2
    )
    metrics = {
        "pressure_ratio": float(aero["pressure_ratio_total_to_total"]),
        "pressure_rise_kpa": float(pressure_rise_kpa),
        "eta_tt_screening": float(aero["eta_tt_screening"]),
        "mass_flow_kg_s": float(aero["mass_flow_kg_s"]),
        "tip_mach": float(aero["tip_mach"]),
        "relative_mach_shroud": float(aero["relative_mach_shroud"]),
        "exit_open_fraction": float(aero["effective_open_fraction_exit"]),
        "screen_safety_factor": float(aero["screen_safety_factor"]),
        "flow_range_penalty": float(flow_penalty),
        "constraint_penalty": float(8.0 * flow_penalty + mach_penalty + open_penalty + shape_penalty),
    }
    return metrics, cfg


def _profile_objective(profile: str, metrics: dict[str, Any]) -> float:
    penalty = float(metrics["constraint_penalty"])
    dp = float(metrics["pressure_rise_kpa"])
    eta = float(metrics["eta_tt_screening"])
    if profile == "max_pressure":
        utility = dp + 3.0 * eta
    elif profile == "max_efficiency":
        # Небольшой член напора не допускает выбора эффективной, но практически
        # бесполезной точки с исчезающе малым pressure rise.
        utility = 100.0 * eta + 0.15 * dp
    elif profile == "balanced":
        # Гармоническое среднее не позволяет одному показателю полностью
        # компенсировать другой. Нормировка соответствует ожидаемому диапазону
        # D200 при 15 000 об/мин и нужна только для ранжирования кандидатов.
        pressure_score = float(np.clip((dp - 15.5) / 4.5, 0.02, 1.20))
        efficiency_score = float(np.clip((eta - 0.700) / 0.045, 0.02, 1.20))
        utility = 2.0 * pressure_score * efficiency_score / (
            pressure_score + efficiency_score
        )
    else:
        raise ValueError(f"Неизвестный профиль оптимизации: {profile}")
    return -utility + 100.0 * penalty


def optimize_profile(
    base: dict[str, Any], profile: str, max_iterations: int | None = None
) -> tuple[DesignConfig, dict[str, Any]]:
    try:
        from scipy.optimize import differential_evolution
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("Для --optimize требуется scipy: pip install scipy") from exc

    evaluations = 0

    def objective(x: np.ndarray) -> float:
        nonlocal evaluations
        evaluations += 1
        try:
            metrics, _ = _optimization_metrics(base, x)
            return _profile_objective(profile, metrics)
        except Exception:
            return 1.0e9

    result = differential_evolution(
        objective,
        bounds=OPTIMIZATION_BOUNDS,
        seed=int(OPTIMIZATION_PARAMETERS["seed"]),
        maxiter=int(max_iterations or OPTIMIZATION_PARAMETERS["max_iterations"]),
        popsize=int(OPTIMIZATION_PARAMETERS["population_size"]),
        tol=2.0e-4,
        polish=True,
        updating="immediate",
        workers=1,
    )
    metrics, cfg = _optimization_metrics(base, result.x)
    throat = ThroatChecker(cfg, AerodynamicCalculator(cfg).calculate()).calculate()
    metrics["minimum_approximate_clearance_mm"] = throat["minimum_approximate_clearance_mm"]

    stand_data = asdict(cfg)
    stand_data["operating"]["rpm"] = float(OPTIMIZATION_PARAMETERS["stand_rpm"])
    stand_cfg = design_from_dict(stand_data)
    stand_aero = AerodynamicCalculator(stand_cfg).calculate()
    stand_dp = (stand_aero["pressure_ratio_total_to_total"] - 1.0) * stand_cfg.operating.p01_pa / 1000.0

    record = {
        "profile": profile,
        "success": bool(result.success),
        "optimizer_message": str(result.message),
        "evaluations": evaluations,
        "objective": float(result.fun),
        "variables": dict(zip(OPTIMIZATION_VARIABLES, [
            cfg.blades.main_blades,
            cfg.geometry.D1_shroud_mm,
            cfg.geometry.b2_mm,
            cfg.blades.splitter_start_fraction,
            cfg.blades.splitter_pitch_fraction,
            cfg.blades.backsweep_from_radial_deg,
            cfg.operating.flow_coefficient_phi2,
            cfg.blades.wrap_scale,
            cfg.blades.loading_bias,
        ])),
        "design_point": metrics,
        "stand_point": {
            "rpm": stand_cfg.operating.rpm,
            "pressure_ratio": stand_aero["pressure_ratio_total_to_total"],
            "pressure_rise_kpa": stand_dp,
            "eta_tt_screening": stand_aero["eta_tt_screening"],
            "mass_flow_kg_s": stand_aero["mass_flow_kg_s"],
        },
        "screening_warning": (
            "Оптимизация mean-line предназначена для выбора кандидатов. "
            "Итоговые напор и КПД подтверждаются CFD, прочность — ANSYS."
        ),
    }
    return cfg, record


def optimize_three_profiles(
    base: dict[str, Any], output_root: str | None, generate_models: bool,
    max_iterations: int | None = None,
) -> dict[str, Any]:
    root = Path(output_root or "impeller_D200_three_optimized_models").expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    configs: dict[str, DesignConfig] = {}
    export_metadata = bool(base["output"].get("export_metadata", False))

    for profile in OPTIMIZATION_PARAMETERS["profiles"]:
        print(f"\n[OPT] Профиль: {profile}")
        cfg, record = optimize_profile(base, profile, max_iterations)
        cfg_data = asdict(cfg)
        cfg_data["output"]["output_directory"] = str(root)
        cfg = design_from_dict(cfg_data)
        configs[profile] = cfg
        if export_metadata:
            config_path = root / f"{profile}_optimized_config.json"
            config_path.write_text(
                json.dumps(asdict(cfg), ensure_ascii=False, indent=2), encoding="utf-8"
            )
            record["config_file"] = str(config_path)
        records.append(record)
        d = record["design_point"]
        print(
            f"      PR={d['pressure_ratio']:.4f}, eta={d['eta_tt_screening']:.4f}, "
            f"mdot={d['mass_flow_kg_s']:.4f} кг/с, Z={cfg.blades.main_blades}"
        )

    summary = {
        "optimization_parameters": copy.deepcopy(OPTIMIZATION_PARAMETERS),
        "profiles": records,
        "note": (
            "Три модели оптимизированы по общей screening-модели и одинаковому "
            "диапазону расхода; абсолютные характеристики требуют CFD."
        ),
    }
    if export_metadata:
        summary_path = root / "optimization_summary.json"
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        with (root / "optimization_comparison.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                "profile", "main_blades", "backsweep_deg", "phi2", "D1_shroud_mm", "b2_mm",
                "pressure_ratio", "pressure_rise_kpa", "eta_tt_screening", "mass_flow_kg_s",
                "stand_pressure_rise_kpa", "min_clearance_mm",
            ])
            for record in records:
                v, d, s = record["variables"], record["design_point"], record["stand_point"]
                w.writerow([
                    record["profile"], v["main_blades"], v["backsweep_from_radial_deg"],
                    v["flow_coefficient_phi2"], v["D1_shroud_mm"], v["b2_mm"],
                    d["pressure_ratio"], d["pressure_rise_kpa"], d["eta_tt_screening"],
                    d["mass_flow_kg_s"], s["pressure_rise_kpa"],
                    d["minimum_approximate_clearance_mm"],
                ])

    if generate_models:
        for profile in OPTIMIZATION_PARAMETERS["profiles"]:
            print(f"\n[CAD] Генерация модели {profile}")
            generated = generate(configs[profile], str(root))
            step_source = Path(generated["files"]["step_fused"])
            step_target = root / f"{profile}.step"
            step_source.replace(step_target)
            # В некоторых синхронизируемых файловых системах replace может
            # оставить исходную копию. Явно удаляем только что созданный
            # промежуточный *_fused.step, чтобы результат содержал ровно 3 STEP.
            if step_source != step_target and step_source.exists():
                step_source.unlink()
            print(f"      итоговый STEP: {step_target.name}")
    print(f"\nТри профиля оптимизации записаны в: {root}")
    return summary


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Параметрический генератор 3D impeller.")
    p.add_argument("--config", help="JSON с переопределением параметров.")
    p.add_argument("--output", help="Папка экспорта.")
    p.add_argument("--quality", choices=sorted(QUALITY), help="draft / normal / fine")
    p.add_argument("--set", nargs="*", default=[], metavar="GROUP.KEY=VALUE")
    p.add_argument("--write-config", help="Записать пример JSON и завершить.")
    p.add_argument("--check-only", action="store_true", help="Только расчёт и проверки без CAD-экспорта.")
    p.add_argument(
        "--optimize", action="store_true",
        help="Оптимизировать и построить три модели: pressure / efficiency / balanced."
    )
    p.add_argument(
        "--optimize-only", action="store_true",
        help="Оптимизировать три профиля и записать конфигурации без CAD-экспорта."
    )
    p.add_argument(
        "--optimization-iterations", type=int,
        help="Число поколений differential evolution (по умолчанию из OPTIMIZATION_PARAMETERS)."
    )
    p.add_argument("--debug", action="store_true", help="Полный traceback при ошибке.")
    return p


def main() -> int:
    args = build_parser().parse_args()
    data = copy.deepcopy(USER_PARAMETERS)
    if args.config:
        data = deep_merge(data, json.loads(Path(args.config).read_text(encoding="utf-8")))
    if args.quality:
        data["output"]["mesh_quality"] = args.quality
    for item in args.set:
        if "=" not in item:
            raise ValueError(f"Ожидалось GROUP.KEY=VALUE, получено: {item}")
        key, value = item.split("=", 1)
        set_dotted(data, key, parse_scalar(value))
    if args.write_config:
        Path(args.write_config).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Конфигурация записана: {args.write_config}")
        return 0
    if args.optimize or args.optimize_only:
        if args.optimization_iterations is not None and args.optimization_iterations < 1:
            raise ValueError("--optimization-iterations должно быть >= 1.")
        optimize_three_profiles(
            data,
            args.output,
            generate_models=bool(args.optimize),
            max_iterations=args.optimization_iterations,
        )
        return 0
    cfg = design_from_dict(data)
    validate_config(cfg)
    if args.check_only:
        aero = AerodynamicCalculator(cfg).calculate()
        throat = ThroatChecker(cfg, aero).calculate()
        print_summary(cfg, aero, throat)
        print(json.dumps({"aerodynamics": aero, "throat": throat}, ensure_ascii=False, indent=2))
        return 0
    generate(cfg, args.output)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nОстановлено пользователем.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nОШИБКА: {exc}", file=sys.stderr)
        if "--debug" in sys.argv:
            traceback.print_exc()
        raise SystemExit(1)
