import math

def calculate_k_member(E, d, D, t):
    """
    使用 Shigley Equation 8-20 计算单个截头圆锥体(Frustum)的刚度。
    
    参数 Parameters:
    E : Young's Modulus (MPa or N/mm^2) - 材料的杨氏模量
    d : Bolt diameter (mm) - 螺栓公称直径
    D : Washer face diameter (mm) - 垫圈面直径/螺栓头接触面直径
    t : Thickness of the plate (mm) - 被夹紧板的厚度
    
    返回 Returns:
    k : Stiffness (N/mm) - 该层材料的刚度
    """
    # 核心公式 Equation 8-20
    # 注意: 1.155 来自 2 * tan(30度)
    # 0.5774 来自 tan(30度)
    
    numerator = 0.5774 * math.pi * E * d
    
    term1 = (1.155 * t + D - d) * (D + d)
    term2 = (1.155 * t + D + d) * (D - d)
    
    # 防止分母为0或负数的安全检查
    if term2 == 0 or (term1/term2) <= 0:
        return float('inf') 
        
    denominator = math.log(term1 / term2)
    
    k = numerator / denominator
    return k

def calculate_total_stiffness(k_list):
    """
    计算多个弹簧串联后的总刚度 1/kt = 1/k1 + 1/k2 + ...
    """
    if not k_list:
        return 0
    
    inverse_sum = sum([1.0/k for k in k_list])
    return 1.0 / inverse_sum

# ==========================================
# 考试时在这里修改输入值 (User Inputs)
# ==========================================

if __name__ == "__main__":
    # 示例: 连接两块板 (Steel plate + Cast Iron plate)
    # 螺栓: M12
    d = 12  # mm
    
    # --- 第一块板 (Top Plate: Steel) ---
    E1 = 207e3  # MPa (Steel)
    t1 = 14.8     # mm (厚度)
    D1 = 19.05   # mm (标准垫圈面直径，如果没有给特定值，通常用 1.5d)
    
    # --- 第二块板 (Bottom Plate: Cast Iron) ---
    E2 = 14.5e6  # MPa (Cast Iron)
    t2 = 0.0775      # mm (厚度)
    D2 = 1.437  # mm (螺母面直径，通常也假设为 1.5d)

    # --- 计算 ---
    print("-" * 30)
    print(f"螺栓直径 d = {d} mm")
    
    # 1. 计算各层刚度
    k1 = calculate_k_member(E1, d, D1, t1)
    print(f"板1刚度 (k1): {k1/1000:.2f} kN/mm  (E={E1/1000} GPa, t={t1} mm)")
    
    k2 = calculate_k_member(E2, d, D2, t2)
    print(f"板2刚度 (k2): {k2/1000:.2f} kN/mm  (E={E2/1000} GPa, t={t2} mm)")
    
    # 2. 计算总刚度 (串联)
    km_total = calculate_total_stiffness([k1, k2])
    print("-" * 30)
    print(f"总被连接件刚度 (km): {km_total:.2f} N/mm")
    print(f"总被连接件刚度 (km): {km_total/1000:.3f} kN/mm")
    print("-" * 30)