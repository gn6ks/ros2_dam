#!/usr/bin/env python3
"""
script de debug para corregir problemas de yaml de ros2
"""

import sys
import os
import tempfile
import shutil
import json

# Intercepta la creación de archivos temporales para ver el YAML
_original_mkstemp = tempfile.mkstemp
_original_NamedTemporaryFile = tempfile.NamedTemporaryFile

def patched_mkstemp(*args, **kwargs):
    fd, path = _original_mkstemp(*args, **kwargs)
    print(f"\n[DEBUG] tempfile.mkstemp created: {path}")
    return fd, path

def patched_NamedTemporaryFile(*args, **kwargs):
    f = _original_NamedTemporaryFile(*args, **kwargs)
    print(f"\n[DEBUG] NamedTemporaryFile created: {f.name}")
    return f

tempfile.mkstemp = patched_mkstemp
tempfile.NamedTemporaryFile = patched_NamedTemporaryFile

# Ahora importa y ejecuta tu código normal
import rclpy
from rclpy.node import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy

# Monkey-patch para capturar el YAML antes de que falle
import launch_param_builder.parameter_builder as pb
_original_load_yaml = pb.load_yaml

def debug_load_yaml(file_path):
    result = _original_load_yaml(file_path)
    print(f"\n[DEBUG] load_yaml({file_path}):")
    print(json.dumps(result, indent=2, default=str)[:2000])
    return result

pb.load_yaml = debug_load_yaml

# Monkey-patch para ver el diccionario final
_original_to_dict = MoveItConfigsBuilder.to_dict

def debug_to_dict(self, *args, **kwargs):
    result = _original_to_dict(self, *args, **kwargs)
    print("\n" + "="*80)
    print("CONFIG DICT COMPLETO:")
    print("="*80)
    
    def safe_dump(obj, indent=0):
        if isinstance(obj, dict):
            for k, v in obj.items():
                print("  " * indent + f"{k}:", end="")
                if isinstance(v, (str, int, float, bool)) or v is None:
                    print(f" {v} (type: {type(v).__name__})")
                elif isinstance(v, list) and len(v) > 0:
                    print(f" [list, len={len(v)}]")
                    # Muestra los tipos de los primeros elementos
                    types = [type(x).__name__ for x in v[:5]]
                    print("  " * (indent+1) + f"types: {types}")
                    for i, item in enumerate(v[:3]):
                        print("  " * (indent+1) + f"[{i}]:")
                        safe_dump(item, indent+2)
                elif isinstance(v, dict):
                    print()
                    safe_dump(v, indent+1)
                else:
                    print(f" {v}")
        elif isinstance(obj, list):
            for i, item in enumerate(obj[:5]):
                print("  " * indent + f"[{i}]:")
                safe_dump(item, indent+1)
    
    safe_dump(result)
    
    # Busca específicamente secuencias con tipos mixtos
    print("\n" + "="*80)
    print("BUSCANDO SECUENCIAS CON TIPOS MIXTOS:")
    print("="*80)
    
    def find_mixed_sequences(obj, path=""):
        if isinstance(obj, list):
            types = set()
            for item in obj:
                if isinstance(item, (int, float)):
                    types.add("number")
                elif isinstance(item, str):
                    types.add("string")
                elif isinstance(item, dict):
                    types.add("dict")
                elif isinstance(item, list):
                    types.add("list")
            if len(types) > 1:
                print(f"\nMIXED TYPES at {path}: {types}")
                print(f"  Values: {obj[:10]}")
            for i, item in enumerate(obj):
                find_mixed_sequences(item, f"{path}[{i}]")
        elif isinstance(obj, dict):
            for k, v in obj.items():
                find_mixed_sequences(v, f"{path}.{k}")
    
    find_mixed_sequences(result)
    
    return result

MoveItConfigsBuilder.to_dict = debug_to_dict

# Ahora ejecuta tu código normal
class MoveGroupPythonIntefaceControl(Node):
    def __init__(self):
        super().__init__("move_group_control", namespace="/lbr")

        moveit_config = (
            MoveItConfigsBuilder("iiwa7", package_name="iiwa7_moveit_config")
            .to_moveit_configs()
        )
        
        print("\n" + "="*80)
        print("PASANDO A MoveItPy...")
        print("="*80)
        
        self._moveit = MoveItPy(config_dict=moveit_config)

def main(args=None):
    rclpy.init(args=args)
    try:
        control = MoveGroupPythonIntefaceControl()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        
        # Intenta leer el último archivo temporal creado
        import glob
        temp_files = glob.glob("/tmp/launch_params_*")
        if temp_files:
            latest = max(temp_files, key=os.path.getctime)
            print(f"\n[DEBUG] Último archivo temporal: {latest}")
            try:
                with open(latest, 'r') as f:
                    content = f.read()
                print(f"\n[DEBUG] Contenido (primeras 30 líneas):")
                for i, line in enumerate(content.split('\n')[:30], 1):
                    marker = "  <-- LÍNEA 7!!!" if i == 7 else ""
                    print(f"  {i:3d}: {line}{marker}")
            except Exception as e2:
                print(f"No se pudo leer: {e2}")
        
        raise
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()