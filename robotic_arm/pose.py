import numpy as np
import math




# ---- Funções básicas SE2 ----
def SE2_xy(x, y):
    """Matriz de translação homogênea 2D (x, y)."""
    return np.array([[1.0, 0.0, x],
                     [0.0, 1.0, y],
                     [0.0, 0.0, 1.0]], dtype=float)

def SE2_theta(theta):
    """Matriz de rotação homogênea 2D (rotaciona por +theta)."""
    c = math.cos(theta)
    s = math.sin(theta)
    # forma padrão: [[cos, -sin],[sin, cos]]
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)

def SE2_pose(x, y, theta):
    """Matriz homogênea da pose: T = trans(x,y) @ rot(theta)
       (gera T tal que p_world = T @ p_local)."""
    return SE2_xy(x, y) @ SE2_theta(theta)

def transform_from_A_to_B(A, B, theta):

    #Informações para as funções
    (xB, yB) = B
    (xA, yA) = A
    (shiftX, shiftY) = (xA - xB, yA - yB)
    return SE2_pose(shiftX, shiftY, theta)

# ---- Operações úteis ----
def inv_SE2(T):
    """Inversa de uma transformação SE2 homogênea.
       inv(T) = [[R^T, -R^T t],[0, 1]]"""
    R = T[0:2, 0:2]
    t = T[0:2, 2].reshape(2, 1)
    R_T = R.T
    t_new = -R_T @ t
    Tinv = np.eye(3, dtype=float)
    Tinv[0:2, 0:2] = R_T
    Tinv[0:2, 2] = t_new.flatten()
    return Tinv

def test1(R1, R2):
    '''Ponto [0.5, 0.5] em R2 -> P em R1
       Angulo entre eles é 0'''
    
    theta = 0
    #Ponto de referencia
    P_R2 = np.array([[0.5], [0.5], [1]])
    
    #Matriz de transformação
    T = transform_from_A_to_B(R2, R1, theta = 0)
    
    #Transformação
    P_R1 = T @ P_R2
    
    #ASSERT com valor téorico
    assert np.allclose(P_R1, np.array([[1.5], [0.75], [1]])), f"Value of P_R1 {P_R1}"# Verifica resultado
    print("Teste 1 Passed")

def test2(R1, R2):
    '''Ponto [0.5, 0.5] em R1 -> P em R2
       Angulo entre eles é 0'''

    theta = 0
    #Ponto de referencia
    P_R1 = np.array([[0.5], [0.5], [1]])
    
    #Matriz de transformação
    T = transform_from_A_to_B(R1, R2, theta = 0)

    #Transformação
    P_R2 = T @ P_R1
    
    #ASSERT com valor téorico
    assert np.allclose(P_R2, np.array([[-0.5], [0.25], [1]])), f"Value of P_R1 {P_R1}"# Verifica resultado
    print("Teste 2 Passed")


def test3(R1, R2):
    '''Ponto [0.5, 0.5] em R2 -> P em R1
       Angulo entre eles é 45'''


    theta = math.pi/4

    #Ponto de referencia
    P_R2 = np.array([[0.5], [0.5], [1]])
    
    #Matriz de transformação
    T = transform_from_A_to_B(R2, R1, theta = theta)

    #Transformação
    P_R1 = T @ P_R2

    #ASSERT com valor téorico
    teoric_y = 1/np.sqrt(2) + 0.25
    assert np.allclose(P_R1, np.array([[1], [teoric_y], [1]])), f"Value of P_R1 {P_R1}"# Verifica resultado
    print("Teste 3 Passed")


def test4(R1, R2):
    '''Ponto [0.5, 0.5] em R1 -> P em R2
       Angulo entre eles é -45
       Fazendo o setup igual o anterior, mas agora, para utilizar a transformada inversa'''

    theta = math.pi/4

    
    #Temos a transformada de R2 -> R1, a invertendo a matriz temos R1 -> R2
    T = transform_from_A_to_B(R2, R1, theta = theta)
    
    #Matriz de transformação
    T_inv = inv_SE2(T)

    #Ponto de referencia
    P_R1 = np.array([[0.5], [0.5], [1]])
    
    #Transformação
    P_R2 = T_inv @ P_R1
    
    #Não foi calculado o téorico, por isso, um print
    print(f"Resultado no teste 4;\n{P_R2}")


# ---- Exemplos / tests ----
if __name__ == "__main__":
    R1 = np.array([0, 0])
    R2 = np.array([1.0, 0.25])

    test1(R1, R2)
    test2(R1, R2)
    test3(R1, R2)
    test4(R1, R2)
