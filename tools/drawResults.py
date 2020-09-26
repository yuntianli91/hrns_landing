#coding=utf-8
######################## 导入模块 #######################
import numpy as np
import pandas as pd
import math
import matplotlib
import matplotlib.pyplot as plt
import systems as sys
from matplotlib import rcParams

# matplotlib.use("pgf")
# pgf_config = {
#     "font.family":'serif',
#     "font.size": 10,
#     "pgf.rcfonts": False,
#     "text.usetex": True,
#     "pgf.preamble": [
#         r"\usepackage{unicode-math}",
#         #r"\setmathfont{XITS Math}", 
#         # 这里注释掉了公式的XITS字体，可以自行修改
#         r"\setmainfont{Times New Roman}",
#         r"\usepackage{xeCJK}",
#         r"\xeCJKsetup{CJKmath=true}",
#         r"\setCJKmainfont{SimSun}",
#     ],
# }
# # rcParams.update(pgf_config
######################## 主函数 ###########################