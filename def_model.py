"""
・モデルの定義(伝達関数・状態空間モデル)
・可観測性・可制御性
・正準系
"""

from control import matlab
import numpy as np

#=======================伝達関数モデル=========================
P = matlab.tf([1,3],[1,5,8,4])
# print(P)

# 同じモデルを掛け算で表現
P1 = matlab.tf([1,3],[1])
P2 = matlab.tf([1],[1,1])
P3 = matlab.tf([1],[1,2])

P = P1 * P2 * P3**2
# print(P)

# 多項式抽出
num = P.num  # 分子多項式
den = P.den  # 分母多項式

pole = matlab.pole(P) # 極
zero = matlab.zero(P) # 零点



#========================状態空間モデル=======================
A = [[1,1,2],[2,1,1],[3,4,5]]
B = [[2],[0],[1]]
C = [[1,1,0]]
D = [[0]]

P = matlab.ss(A, B, C, D)
# print(P)

# 行列抽出
# print("A=", P.A)   # 方法１
sysA, sysB, sysC, sysD = matlab.ssdata(P)  # 方法２
# print("A=",sysA)




#==========================ブロック線図=======================

#-------> s3 -------------------> s1 ------------> y
#    ^            +   ^                    |
#    |              - |                    |
#    |                ----------- s2 <------ 
#    |                                     |
#     ----------------------------s4 <------

S1 = matlab.tf([1],[1,1])
S2 = matlab.tf([1],[1,2])
S3 = matlab.tf([3,1],[1,0])
S4 = matlab.tf([2,0],[1])

S12 = matlab.feedback(S1, S2)
S123 = matlab.series(S12, S3)
S = matlab.feedback(S123, S4)
# print(S)


#       |-------> S1 ----|
# ------|                |-----> y
#       |-------> S2 ----|
# 並列時は
# matlab.parallel(S1, S2)をつかう




# ==================伝達関数と状態空間表現の変換=========================
# 状態空間モデルから伝達関数モデルへの変換は一意である。
# 　　　　　　　　G(s) = C(sI-A)^-1 B
# しかし伝達関数モデルから状態変数モデルへの変換は無数にある。
# これは状態空間モデルの状態を自由に選ぶことができるためである。
# そのため可制御正準系、可観測正準系で表現することが多い。

P = matlab.tf([1],[1,1,1])

Pss = matlab.tf2ss(P) # 伝達関数モデルから状態空間モデルへの変換 (可制御正準系や可観測正準系ではない可能性がある)
# print(Pss)
Ptf = matlab.ss2tf(Pss) # 状態空間モデルから伝達関数モデルへの変換
# print(Ptf)


# ========================可制御性・可観測性=================================
A = [[0,1],[-4,5]]
B = [[0],[1]]
C = [[1,0]]
D = [[0]]

P = matlab.ss(A, B, C, D)

Uc = matlab.ctrb(P.A, P.B) # Uc:可制御性行列　　[[B][AB][A^2 B] ...]
# rank Uc = n  <=>  |Uc| != 0  なら可制御

# print("Uc=\n",Uc)
# print("det(Uc)=",np.linalg.det(Uc))
# print("rank(Uc)=",np.linalg.matrix_rank(Uc))
# if np.linalg.det(Uc) != 0:
#     print("可制御")

Uo = matlab.obsv(P.A, P.C) # Uo:可観測性行列   [[C][CA][CA^2] ...].T
# rank Uo = n  <=>  |Uo| != 0  なら可観測

# print("Uo=\n",Uo)
# print("det(Uo)=",np.linalg.det(Uo))
# print("rank(Uo)=",np.linalg.matrix_rank(Uo))
# if np.linalg.det(Uo) != 0:
#     print("可観測")





# ==================可制御正準系・可観測正準系===========================
from control import canonical_form
A = [[1,2,3],[3,2,1],[4,5,0]]
B = [[1],[0],[1]]
C = [[0,2,1]]
D = [[0]]
Pss = matlab.ss(A, B, C, D)

Pr, T = canonical_form(Pss, form='reachable') # 可制御正準系
# print(Pr)
Po, T = canonical_form(Pss, form='observable') # 可観測正準系
# print(Po)