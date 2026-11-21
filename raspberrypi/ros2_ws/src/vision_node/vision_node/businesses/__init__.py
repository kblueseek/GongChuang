"""自动加载本目录下所有业务模块。

新增视觉动作只需要在这里放一个 .py 文件，并在类上写 @register；
不需要改 node.py，也不需要改这个文件。
"""

import importlib
import pkgutil

for _module in pkgutil.iter_modules(__path__):
    importlib.import_module(f'{__name__}.{_module.name}')
