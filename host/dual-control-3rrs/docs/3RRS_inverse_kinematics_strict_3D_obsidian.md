# 3-RRS 并联板球平台严格三维闭环逆运动学

> 本文以 Ashrafi 与 Moosavian（2024）《Kinematic and Kinetic Analysis of 3-RRS Parallel Manipulator》中 Eq. (1)–(15) 的 3-RRS 几何模型为主线。
>
> 与此前版本不同，本文**不再先把平台连接点投影成二维坐标再求解**，而是从每条 R-R-S 支链的完整三维闭环矢量方程出发，并显式检查支链的切向闭合约束。
>
> 另外，论文 Eq. (15) 的符号与论文自身 Eq. (3)–(4)、Eq. (16) 以及 Eq. (28) 存在内部不一致。本文会把这一点明确推导出来，并采用满足完整闭环的符号。

---

## 1. 机构、坐标系与独立变量

3-RRS 并联机构由三条相隔 $120^\circ$ 的 R-R-S 支链构成。

固定坐标系记为 $O_0-XYZ$，运动平台坐标系记为 $P-xyz$。

三条支链的安装方位角为

$$
\boxed{
\alpha_1=0,\qquad
\alpha_2=\frac{2\pi}{3},\qquad
\alpha_3=\frac{4\pi}{3}
}
$$

平台中心在固定坐标系中的位置为

$$
\mathbf p=
\begin{bmatrix}
X_p\\
Y_p\\
Z_p
\end{bmatrix}.
$$

论文规定真正独立的任务空间变量只有

$$
\boxed{
\mathbf q_t=
\begin{bmatrix}
\psi_x\\
\psi_y\\
Z_p
\end{bmatrix}
}
$$

而

$$
X_p,\qquad Y_p,\qquad \psi_z
$$

均由机构闭环约束决定。

逆运动学的目标是求三个主动关节角

$$
\boxed{
\boldsymbol\theta=
\begin{bmatrix}
\theta_1\\
\theta_2\\
\theta_3
\end{bmatrix}.
}
$$

---

## 2. 平台旋转矩阵

按照论文 Eq. (1)，平台依次绕固定坐标系的 $Z\rightarrow Y\rightarrow X$ 方向旋转。

总旋转矩阵写成

$$
R=R_x(\psi_x)R_y(\psi_y)R_z(\psi_z).
$$

定义

$$
c_X=\cos\psi_x,\quad s_X=\sin\psi_x,
$$

$$
c_Y=\cos\psi_y,\quad s_Y=\sin\psi_y,
$$

$$
c_Z=\cos\psi_z,\quad s_Z=\sin\psi_z.
$$

则

$$
\boxed{
R=
\begin{bmatrix}
c_Yc_Z & -c_Ys_Z & s_Y\\
c_Zs_Xs_Y+c_Xs_Z &
c_Xc_Z-s_Xs_Ys_Z &
-s_Xc_Y\\
s_Xs_Z-c_Xs_Yc_Z &
s_Xc_Z+c_Xs_Ys_Z &
c_Xc_Y
\end{bmatrix}.
}
$$

记

$$
R=
\begin{bmatrix}
r_{11}&r_{12}&r_{13}\\
r_{21}&r_{22}&r_{23}\\
r_{31}&r_{32}&r_{33}
\end{bmatrix}.
$$

---

## 3. 平台三个球铰连接点

三个连接点在平台坐标系中的坐标为

$$
{}^P\mathbf r_i
=
r_p
\begin{bmatrix}
\cos\alpha_i\\
\sin\alpha_i\\
0
\end{bmatrix}.
$$

因此其固定坐标系坐标为

$$
\boxed{
\mathbf A_i
=
\mathbf p+R\,{}^P\mathbf r_i.
}
$$

统一展开：

$$
\boxed{
\mathbf A_i
=
\begin{bmatrix}
X_p+r_p(r_{11}\cos\alpha_i+r_{12}\sin\alpha_i)\\
Y_p+r_p(r_{21}\cos\alpha_i+r_{22}\sin\alpha_i)\\
Z_p+r_p(r_{31}\cos\alpha_i+r_{32}\sin\alpha_i)
\end{bmatrix}.
}
$$

具体为

$$
\mathbf A_1=
\begin{bmatrix}
X_p+r_pr_{11}\\
Y_p+r_pr_{21}\\
Z_p+r_pr_{31}
\end{bmatrix},
$$

$$
\mathbf A_2=
\begin{bmatrix}
X_p-\frac{r_p}{2}r_{11}+\frac{\sqrt3r_p}{2}r_{12}\\
Y_p-\frac{r_p}{2}r_{21}+\frac{\sqrt3r_p}{2}r_{22}\\
Z_p-\frac{r_p}{2}r_{31}+\frac{\sqrt3r_p}{2}r_{32}
\end{bmatrix},
$$

$$
\mathbf A_3=
\begin{bmatrix}
X_p-\frac{r_p}{2}r_{11}-\frac{\sqrt3r_p}{2}r_{12}\\
Y_p-\frac{r_p}{2}r_{21}-\frac{\sqrt3r_p}{2}r_{22}\\
Z_p-\frac{r_p}{2}r_{31}-\frac{\sqrt3r_p}{2}r_{32}
\end{bmatrix}.
$$

这与论文 Eq. (28)–(30) 的球铰位置表达一致。

---

## 4. 每条 R-R-S 支链的完整三维闭环

论文 Eq. (2) 给出

$$
\boxed{
\mathbf r_3+\mathbf r_{pi}
=
\mathbf r_{0i}+\mathbf r_{1i}+\mathbf r_{2i},
\qquad i=1,2,3.
}
$$

左边就是球铰点位置 $\mathbf A_i$，因此可写成

$$
\boxed{
\mathbf A_i
=
\mathbf r_{0i}+\mathbf r_{1i}+\mathbf r_{2i}.
}
$$

第 $i$ 条支链的基座位置向量：

$$
\mathbf r_{0i}
=
r_0
\begin{bmatrix}
\cos\alpha_i\\
\sin\alpha_i\\
0
\end{bmatrix}.
$$

主动连杆长度为 $L_1$，主动关节角为 $\theta_i$：

$$
\mathbf r_{1i}
=
L_1
\begin{bmatrix}
\cos\theta_i\cos\alpha_i\\
\cos\theta_i\sin\alpha_i\\
-\sin\theta_i
\end{bmatrix}.
$$

被动连杆长度为 $L_2$，被动关节角为 $\phi_i$：

$$
\mathbf r_{2i}
=
L_2
\begin{bmatrix}
\cos\phi_i\cos\alpha_i\\
\cos\phi_i\sin\alpha_i\\
-\sin\phi_i
\end{bmatrix}.
$$

所以真正的三维闭环为

$$
\boxed{
\mathbf A_i
=
\begin{bmatrix}
\cos\alpha_i
\left(
r_0+L_1\cos\theta_i+L_2\cos\phi_i
\right)\\
\sin\alpha_i
\left(
r_0+L_1\cos\theta_i+L_2\cos\phi_i
\right)\\
-L_1\sin\theta_i-L_2\sin\phi_i
\end{bmatrix}.
}
$$

其三个标量分量为

$$
\boxed{
L_2\cos\phi_i\cos\alpha_i
=
A_{i,X}
-
\cos\alpha_i
\left(
r_0+L_1\cos\theta_i
\right)
}
$$

$$
\boxed{
L_2\cos\phi_i\sin\alpha_i
=
A_{i,Y}
-
\sin\alpha_i
\left(
r_0+L_1\cos\theta_i
\right)
}
$$

$$
\boxed{
L_2\sin\phi_i
=
-A_{i,Z}-L_1\sin\theta_i.
}
$$

这正是论文 Eq. (3)–(5)。

---

## 5. 不能丢掉的切向闭合约束

前两个闭环方程中的 $X,Y$ 分量并不是两个互相独立的长度关系。

因为两根 R 连杆始终处在方位角为 $\alpha_i$ 的同一个竖直平面内，所以球铰点 $\mathbf A_i$ 必须也位于该平面中。

定义第 $i$ 条支链的径向单位向量

$$
\mathbf e_{r,i}
=
\begin{bmatrix}
\cos\alpha_i\\
\sin\alpha_i\\
0
\end{bmatrix}
$$

以及水平切向单位向量

$$
\mathbf e_{t,i}
=
\begin{bmatrix}
-\sin\alpha_i\\
\cos\alpha_i\\
0
\end{bmatrix}.
$$

完整闭环必须满足

$$
\boxed{
g_i
=
\mathbf e_{t,i}^{T}\mathbf A_i
=
-A_{i,X}\sin\alpha_i
+
A_{i,Y}\cos\alpha_i
=
0.
}
$$

这是此前二维投影版本中被隐含掉的条件。

只有当

$$
\boxed{g_i=0}
$$

严格成立时，才允许把三维支链看成其自身竖直平面中的二维机构。

因此在数值实现中，必须首先检查

$$
\boxed{
|g_i|<\varepsilon_g.
}
$$

如果 $g_i\neq0$，即使投影后的二维长度方程存在解，该解也**不能闭合真实 R-R-S 支链**。

---

## 6. 从三条切向约束求 $X_p,Y_p,\psi_z$

将

$$
\mathbf A_i=\mathbf p+R\,{}^P\mathbf r_i
$$

代入

$$
g_i=0
$$

并分别取

$$
\alpha_1=0,\qquad
\alpha_2=\frac{2\pi}{3},\qquad
\alpha_3=\frac{4\pi}{3},
$$

三条切向约束可以化为

$$
\boxed{
Y_p=-r_pr_{21}
}
$$

$$
\boxed{
X_p=\frac{r_p}{2}(r_{11}-r_{22})
}
$$

以及

$$
\boxed{
r_{12}=r_{21}.
}
$$

其中第三个条件决定平台的从属偏航角 $\psi_z$。

由旋转矩阵

$$
r_{12}=-c_Ys_Z,
$$

$$
r_{21}=c_Zs_Xs_Y+c_Xs_Z.
$$

令

$$
r_{12}=r_{21},
$$

得到

$$
-c_Ys_Z=c_Zs_Xs_Y+c_Xs_Z.
$$

因此

$$
-(c_X+c_Y)s_Z=c_Zs_Xs_Y
$$

从而

$$
\boxed{
\tan\psi_z
=
-\frac{s_Xs_Y}{c_X+c_Y}.
}
$$

这与论文 Eq. (13) 一致。

数值实现中建议直接写成

$$
\boxed{
\psi_z
=
\operatorname{atan2}
\left(
-s_Xs_Y,\,
c_X+c_Y
\right).
}
$$

由

$$
X_p=\frac{r_p}{2}(r_{11}-r_{22})
$$

以及

$$
r_{11}=c_Yc_Z,
$$

$$
r_{22}=c_Xc_Z-s_Xs_Ys_Z,
$$

得到

$$
\boxed{
X_p
=
\frac{r_p}{2}
\left(
c_Yc_Z-c_Xc_Z+s_Xs_Ys_Z
\right),
}
$$

与论文 Eq. (14) 一致。

而 $Y_p$ 必须满足

$$
\boxed{
Y_p
=
-r_p
\left(
c_Zs_Xs_Y+c_Xs_Z
\right).
}
$$

### 6.1 关于论文 Eq. (15) 的符号

论文 Eq. (15) 印刷为

$$
Y_p
=
+r_p
\left(
c_Zs_Xs_Y+c_Xs_Z
\right).
$$

但在论文自身其余方程采用同一坐标定义时，这个正号无法满足完整闭环。

理由一：论文 Eq. (28) 给出

$$
A_{1,Y}=Y_p+r_pr_{21}.
$$

而论文 Eq. (16) 对同一球铰点给出

$$
A_{1,Y}=0.
$$

因此必须有

$$
\boxed{
Y_p=-r_pr_{21}.
}
$$

理由二：由论文 Eq. (3)–(4) 直接消去共同的
$r_0+L_1\cos\theta_i+L_2\cos\phi_i$，
也会得到

$$
-A_{i,X}\sin\alpha_i+A_{i,Y}\cos\alpha_i=0,
$$

再与 $\mathbf A_i=\mathbf p+R\,{}^P\mathbf r_i$ 联立，同样得到

$$
Y_p=-r_pr_{21}.
$$

因此，**在本文采用的、与论文 Eq. (2)–(5)、Eq. (16)、Eq. (28)–(30) 一致的坐标约定下，严格闭环实现应使用负号。**

如果代码直接照抄论文 Eq. (15) 的正号，则第 1 条支链会产生

$$
A_{1,Y}
=
2r_pr_{21},
$$

即非零切向偏移，从而出现类似

```text
source IK does not close a real R-R-S chain
```

的闭环失败。

---

## 7. 在完整三维闭环下消去被动关节 $\phi_i$

现在保留论文完整闭环 Eq. (3)–(5)。

取 $X$ 分量：

$$
L_2\cos\phi_i\cos\alpha_i
=
A_{i,X}
-
\cos\alpha_i
(r_0+L_1\cos\theta_i).
$$

取 $Z$ 分量：

$$
L_2\sin\phi_i
=
-A_{i,Z}-L_1\sin\theta_i.
$$

将第二式乘以 $\cos\alpha_i$，再将两式平方相加。

因为

$$
\cos^2\phi_i+\sin^2\phi_i=1,
$$

得到

$$
\left[
A_{i,X}
-
\cos\alpha_i
(r_0+L_1\cos\theta_i)
\right]^2
+
\cos^2\alpha_i
\left[
A_{i,Z}+L_1\sin\theta_i
\right]^2
=
L_2^2\cos^2\alpha_i.
$$

展开并整理成

$$
\boxed{
D_i\cos\theta_i
+
E_i\sin\theta_i
+
F_i
=0.
}
$$

其中完全采用论文 Eq. (7)–(9)：

$$
\boxed{
D_i
=
2L_1\cos\alpha_i
\left(
r_0\cos\alpha_i-A_{i,X}
\right)
}
$$

$$
\boxed{
E_i
=
2L_1A_{i,Z}\cos^2\alpha_i
}
$$

$$
\boxed{
F_i
=
A_{i,X}^2
-
2r_0A_{i,X}\cos\alpha_i
+
\cos^2\alpha_i
\left(
r_0^2+L_1^2-L_2^2+A_{i,Z}^2
\right).
}
$$

这里没有把 $\mathbf A_i$ 预先投影到二维平面。

$A_{i,X},A_{i,Y},A_{i,Z}$ 始终来自完整的

$$
\boxed{
\mathbf A_i=\mathbf p+R\,{}^P\mathbf r_i,
}
$$

并且 $A_{i,Y}$ 通过前面的

$$
g_i=0
$$

参与完整三维闭环验证。

---

## 8. 主动关节角 $\theta_i$

论文 Eq. (6) 已经把每条支链化为

$$
D_i\cos\theta_i
+
E_i\sin\theta_i
+
F_i
=0.
$$

为了避免半角公式的分支与符号处理，可以直接令

$$
\rho_i
=
\sqrt{D_i^2+E_i^2},
$$

$$
\delta_i
=
\operatorname{atan2}(E_i,D_i).
$$

于是

$$
D_i=\rho_i\cos\delta_i,
$$

$$
E_i=\rho_i\sin\delta_i.
$$

所以

$$
D_i\cos\theta_i+E_i\sin\theta_i
=
\rho_i\cos(\theta_i-\delta_i).
$$

原方程变成

$$
\rho_i\cos(\theta_i-\delta_i)+F_i=0.
$$

因此

$$
\cos(\theta_i-\delta_i)
=
-\frac{F_i}{\rho_i}.
$$

最终得到

$$
\boxed{
\theta_i
=
\operatorname{atan2}(E_i,D_i)
\pm
\arccos
\left(
-\frac{F_i}{\sqrt{D_i^2+E_i^2}}
\right),
\qquad i=1,2,3.
}
$$

这是直接对论文 Eq. (6) 的等价求根，不改变论文 Eq. (7)–(9) 的几何模型。

存在实数解的必要条件为

$$
\boxed{
D_i^2+E_i^2-F_i^2\ge0.
}
$$

等价地，

$$
\boxed{
\left|
\frac{F_i}{\sqrt{D_i^2+E_i^2}}
\right|
\le1.
}
$$

三个支链各自有两个候选解，因此理论上共有

$$
\boxed{
2^3=8
}
$$

种主动关节组合，与论文的结论一致。

实际机构只应选择与真实装配模式一致，并且与上一时刻关节角连续的分支。

---

## 9. 关于论文 Eq. (10) 的实现注意

论文 Eq. (10) 印刷为

$$
\theta_i
=
2\tan^{-1}
\left(
\frac{
-E_i\pm\sqrt{D_i^2+E_i^2-F_i^2}
}{
D_i-F_i
}
\right).
$$

但是如果直接从论文 Eq. (6)

$$
D_i\cos\theta_i+E_i\sin\theta_i+F_i=0
$$

使用标准半角代换

$$
t=\tan\frac{\theta_i}{2},
$$

$$
\cos\theta_i=\frac{1-t^2}{1+t^2},
\qquad
\sin\theta_i=\frac{2t}{1+t^2},
$$

则得到

$$
(F_i-D_i)t^2+2E_it+(D_i+F_i)=0.
$$

所以标准代数展开给出

$$
t=
\frac{
-E_i\pm\sqrt{D_i^2+E_i^2-F_i^2}
}{
F_i-D_i
}.
$$

也就是说，论文 Eq. (10) 的分母符号与直接从 Eq. (6) 进行的标准半角推导并不一致。

因此工程实现建议直接使用上一节的

$$
\boxed{
\operatorname{atan2}+\arccos
}
$$

形式，并最终用论文 Eq. (6) 的残差检查候选解，而不要只依赖 Eq. (10) 的印刷形式。

---

## 10. 被动关节 $\phi_i$ 与最终三维闭环验证

求得 $\theta_i$ 后，由论文 Eq. (11)：

$$
\boxed{
\cos\phi_i
=
\frac{
A_{i,X}
-
\cos\alpha_i
(r_0+L_1\cos\theta_i)
}{
L_2\cos\alpha_i
}
}
$$

$$
\boxed{
\sin\phi_i
=
-\frac{
A_{i,Z}+L_1\sin\theta_i
}{
L_2
}.
}
$$

按 MATLAB / C++ 常见的 `atan2(y,x)` 约定，推荐计算

$$
\boxed{
\phi_i
=
\operatorname{atan2}
(\sin\phi_i,\cos\phi_i).
}
$$

然后重新构造整条支链：

$$
\widehat{\mathbf A}_i
=
\mathbf r_{0i}
+
\mathbf r_{1i}
+
\mathbf r_{2i}.
$$

定义完整三维闭环残差

$$
\boxed{
\mathbf e_i
=
\mathbf A_i-\widehat{\mathbf A}_i.
}
$$

最终必须满足

$$
\boxed{
\|\mathbf e_i\|\approx0.
}
$$

同时检查切向残差

$$
\boxed{
g_i
=
-A_{i,X}\sin\alpha_i
+
A_{i,Y}\cos\alpha_i
\approx0.
}
$$

以及主动关节标量方程残差

$$
\boxed{
r_{\theta,i}
=
D_i\cos\theta_i
+
E_i\sin\theta_i
+
F_i
\approx0.
}
$$

只有三类残差全部通过，才能认为这个候选解真正闭合了三维 R-R-S 支链。

---

## 11. 严格逆运动学完整计算链

输入：

$$
\boxed{
(\psi_x,\psi_y,Z_p)
}
$$

第一步，由切向闭环约束求从属偏航：

$$
\boxed{
\psi_z
=
\operatorname{atan2}
(-s_Xs_Y,c_X+c_Y).
}
$$

第二步，求从属平移：

$$
\boxed{
X_p
=
\frac{r_p}{2}
\left(
c_Yc_Z-c_Xc_Z+s_Xs_Ys_Z
\right)
}
$$

$$
\boxed{
Y_p
=
-r_p
\left(
c_Zs_Xs_Y+c_Xs_Z
\right).
}
$$

第三步，计算

$$
R(\psi_x,\psi_y,\psi_z).
$$

第四步，对 $i=1,2,3$ 计算完整空间球铰点

$$
\boxed{
\mathbf A_i
=
\mathbf p
+
R\,{}^P\mathbf r_i.
}
$$

第五步，首先验证支链平面闭合：

$$
\boxed{
g_i
=
-A_{i,X}\sin\alpha_i
+
A_{i,Y}\cos\alpha_i
\approx0.
}
$$

第六步，利用论文 Eq. (7)–(9) 计算

$$
D_i,\quad E_i,\quad F_i.
$$

第七步，计算两个主动关节候选解

$$
\boxed{
\theta_i
=
\operatorname{atan2}(E_i,D_i)
\pm
\arccos
\left(
-\frac{F_i}{\sqrt{D_i^2+E_i^2}}
\right).
}
$$

第八步，对所有候选组合计算 $\phi_i$，并执行完整三维闭环残差检查：

$$
\boxed{
\mathbf A_i
-
(
\mathbf r_{0i}+\mathbf r_{1i}+\mathbf r_{2i}
)
\approx\mathbf0.
}
$$

最终选择满足机械关节范围、装配模式和时间连续性的唯一物理解。

因此严格计算流程为

$$
\boxed{
(\psi_x,\psi_y,Z_p)
\rightarrow
(\psi_z,X_p,Y_p)
\rightarrow
R
\rightarrow
\mathbf A_i
\rightarrow
g_i
\rightarrow
(D_i,E_i,F_i)
\rightarrow
\theta_i
\rightarrow
\phi_i
\rightarrow
\text{3D closure verification}.
}
$$

最终得到

$$
\boxed{
(\psi_x,\psi_y,Z_p)
\longrightarrow
(\theta_1,\theta_2,\theta_3).
}
$$

---

## 12. 对 Simscape / MATLAB 实现的直接要求

实现时不要再采用“先将 $\mathbf A_i$ 投影到支链平面，然后无条件求二维 IK”的流程。

正确逻辑应该是：

```text
输入 roll / pitch / Zp
        |
        v
由严格闭环求 psi_z, Xp, Yp
        |
        v
计算 R 和完整三维 A1,A2,A3
        |
        v
检查每条腿的 tangential residual g_i
        |
        +---- g_i 过大 ----> 立即判定 pose/公式/坐标约定错误
        |
        v
由论文 Eq.(7)-(9) 求 D,E,F
        |
        v
求 theta_i 两个候选解
        |
        v
求 phi_i
        |
        v
重建 r0i+r1i+r2i
        |
        v
检查完整 3D closure residual
        |
        v
选择唯一物理分支
```

推荐容差根据模型尺度设置，例如双精度纯数学验证时可从

$$
10^{-10}\sim10^{-8}\ {\rm m}
$$

量级开始；Simscape 数值仿真则根据求解器设置适当放宽。

最重要的是：

$$
\boxed{
\text{不要把 }g_i\neq0\text{ 的目标点通过投影“修正”成一个看似可解的二维点。}
}
$$

因为那样求出的 $\theta_i$ 不再对应真实的三维 R-R-S 闭环。
