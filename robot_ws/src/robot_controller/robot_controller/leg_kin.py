import numpy as np


class LEG_KIN:
  # TODO: did this at midnight, something is wrong just cant find it
  # using it for now :/
  def __init__(self) -> None:
    self.shoulder_len = 0.038
    self.a_len = 0.1059
    self.b_len = 0.0245
    self.c_len = 0.047434
    self.d_len = 0.063725
    self.e_len = 0.11058
    pass

  def leg_ik(self, relative_x: float, relative_y: float, relative_z: float, right: bool) -> tuple:
    theta_0, target_z = self._yz_plane_ik(relative_y, relative_z, right)
    theta_1, theta_2 = self._xz_plane_ik(relative_x, target_z)

    return (theta_0, theta_1, theta_2)
  
  def leg_control_conv(self, theta_0: float, theta_1:float, theta_2: float) -> list:
    theta_1_prime = theta_1 - np.pi/2
    upper_inner = np.pi - theta_1 + theta_2
    alpha = (self.a_len ** 2 + self.b_len ** 2 - 2 * self.a_len * self.b_len * np.cos(upper_inner)) ** (1/2)
    beta_upper = np.arccos((self.b_len ** 2 - self.a_len ** 2 - alpha ** 2) / (-2 * self.a_len * alpha)) 
    beta_lower = np.arccos((self.e_len ** 2 - self.c_len ** 2 - alpha ** 2) / (-2 * self.c_len * alpha))
    beta = beta_upper + beta_lower
    theta_2_prime = np.pi - beta

    return (theta_0, theta_1_prime, theta_2_prime)

  def _yz_plane_ik (self, relative_y: float, relative_z: float, right: bool) -> tuple:
    relative_dist_sqr = relative_y ** 2 + relative_z ** 2

    target_len = (relative_dist_sqr - self.shoulder_len ** 2) ** (1/2)
    if (relative_z < 0):
      target_len *= -1

    phi_0 = np.arccos(self.shoulder_len / (relative_dist_sqr ** (1/2)))
    
    if right:
      phi = np.arctan2(relative_z, relative_y)
    else:
      phi = np.arctan2(relative_z, -relative_y)

    if phi < 0:
      phi += 2 * np.pi

    phi_prime = phi - np.pi
    theta = phi_0 - phi_prime

    return (theta, target_len)
  
  def _xz_plane_ik(self, relative_x: float, relative_z: float) -> tuple:
    relative_dist_sqr = relative_x ** 2 + relative_z ** 2

    f = self.c_len + self.d_len

    phi_prime_num = f ** 2 - self.a_len ** 2 - relative_dist_sqr
    phi_prime_den = -2 * self.a_len * relative_dist_sqr**(1/2)
    phi_prime = np.arccos(phi_prime_num/ phi_prime_den)
    
    beta_numerator = relative_dist_sqr - self.a_len ** 2 - f ** 2
    beta_denominator = - 2 * self.a_len * f
    beta = np.arccos(beta_numerator / beta_denominator)
    
    phi = np.arctan2(relative_z, relative_x)

    if phi < 0:
      phi += 2 * np.pi

    theta_1 = phi - np.pi - phi_prime
    
    internal_len_sqr = self.a_len ** 2 + self.c_len ** 2 - 2 * self.a_len * self.c_len * np.cos(beta)
    phi_1_prime_num = self.e_len ** 2 - internal_len_sqr - self.b_len ** 2
    phi_1_prime_den = - 2 * internal_len_sqr ** (1/2) * self.b_len
    phi_1_prime = np.arccos(phi_1_prime_num / phi_1_prime_den)
    phi_1 = 2 * np.pi - phi

    correction_num = self.d_len ** 2 - relative_dist_sqr - internal_len_sqr
    correction_den = - 2 * relative_dist_sqr ** (1/2) * internal_len_sqr ** (1/2)
    correction = np.arccos(correction_num/correction_den)

    theta_2 = phi_1 - phi_1_prime + correction

    return (theta_1, -theta_2)

  def _len(self, point: tuple):
    return (point[0]**2, point[1]**2)**(1/2) 
