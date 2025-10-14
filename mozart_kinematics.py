"""
Mozart Robot Forward Kinematics Module
Converted from MATLAB implementation
"""

import torch
import math
import numpy as np


class MozartKinematics:
    """Forward kinematics computation for the Mozart robot"""
    
    def __init__(self, device='cuda'):
        self.device = device
        
        # Robot parameters - from the original MATLAB code
        self.r = 0.04406  # Radius of the inscribed circle of the base hexagon (m)
        self.l = 0.065    # Leg length (m)
        self.Ob = torch.tensor([0.0, 0.0, 0.0], device=device)  # Base center position
        
        # Angles for the virtual joint centers - corresponding to theta1, theta2, theta3
        self.theta1 = math.pi / 3      # 60 degrees
        self.theta2 = math.pi          # 180 degrees  
        self.theta3 = 5 * math.pi / 3  # 300 degrees
        
        print(f"🤖 Mozart Kinematics Module Initialized:")
        print(f"   📏 r = {self.r:.5f}m (hexagon inscribed circle radius)")
        print(f"   📏 l = {self.l:.3f}m (leg length)")
        print(f"   📐 θ angles = [{math.degrees(self.theta1):.0f}°, {math.degrees(self.theta2):.0f}°, {math.degrees(self.theta3):.0f}°]")
    
    def forward_kinematics_single(self, phi1, phi2, phi3):
        """
        Single forward kinematics computation - corresponds directly to the MATLAB function.
        
        Args:
            phi1, phi2, phi3: Joint angles (radians, typically between 0 and π/2)
            
        Returns:
            pos: End-effector position [x, y, z]
            N: Plane normal vector [nx, ny, nz]
            azimuth: Azimuth angle (degrees)
            elevation: Elevation angle (degrees)
            r_spherical: Spherical radius
        """
        # Ensure inputs are tensors on the correct device
        if not isinstance(phi1, torch.Tensor):
            phi1 = torch.tensor(phi1, device=self.device, dtype=torch.float32)
        if not isinstance(phi2, torch.Tensor):
            phi2 = torch.tensor(phi2, device=self.device, dtype=torch.float32)
        if not isinstance(phi3, torch.Tensor):
            phi3 = torch.tensor(phi3, device=self.device, dtype=torch.float32)
        
        # Compute joint sphere center positions - directly from MATLAB code
        # b1=[cos(theta1)*(r+l*cos(phi1)),sin(theta1)*(r+l*cos(phi1)),l*sin(phi1)];
        cos_theta1 = torch.cos(torch.tensor(self.theta1, device=self.device))
        sin_theta1 = torch.sin(torch.tensor(self.theta1, device=self.device))
        b1 = torch.stack([
            cos_theta1 * (self.r + self.l * torch.cos(phi1)),
            sin_theta1 * (self.r + self.l * torch.cos(phi1)),
            self.l * torch.sin(phi1)
        ])
        
        cos_theta2 = torch.cos(torch.tensor(self.theta2, device=self.device))
        sin_theta2 = torch.sin(torch.tensor(self.theta2, device=self.device))
        b2 = torch.stack([
            cos_theta2 * (self.r + self.l * torch.cos(phi2)),
            sin_theta2 * (self.r + self.l * torch.cos(phi2)),
            self.l * torch.sin(phi2)
        ])
        
        cos_theta3 = torch.cos(torch.tensor(self.theta3, device=self.device))
        sin_theta3 = torch.sin(torch.tensor(self.theta3, device=self.device))
        b3 = torch.stack([
            cos_theta3 * (self.r + self.l * torch.cos(phi3)),
            sin_theta3 * (self.r + self.l * torch.cos(phi3)),
            self.l * torch.sin(phi3)
        ])
        
        # Compute the normal vector of the virtual plane: N = cross((b1-b2), (b1-b3))
        v1 = b1 - b2
        v2 = b1 - b3
        N = torch.cross(v1, v2, dim=0)
        
        # Normalize N
        N_norm = torch.norm(N)
        N_normalized = N / (N_norm + 1e-8)  # Avoid division by zero
        
        # Distance from Ob to the plane: d = dot((b1-Ob), N) / norm(N)
        d = torch.dot((b1 - self.Ob), N) / N_norm
        
        # End-effector position: pos = 2 * d * N / norm(N)
        pos = 2 * d * N_normalized
        
        # Convert to spherical coordinates: [azimuth, elevation, r] = cart2sph(N(1),N(2),N(3))
        x, y, z = pos[0], pos[1], pos[2]
        r_spherical = torch.norm(pos)
        
        # Compute azimuth and elevation
        azimuth = torch.atan2(y, x) * 180.0 / math.pi  # degrees
        elevation = torch.asin(z / (r_spherical + 1e-8)) * 180.0 / math.pi * 2  # *2 follows MATLAB convention
        
        return pos, N_normalized, azimuth, elevation, r_spherical
    
    def forward_kinematics_batch(self, phi_batch):
        """
        Batch forward kinematics computation
        
        Args:
            phi_batch: tensor of shape (batch_size, 3) or (3,)
            
        Returns:
            positions: (batch_size, 3) or (3,)
            normals: (batch_size, 3) or (3,)
            azimuths: (batch_size,) or scalar
            elevations: (batch_size,) or scalar  
            r_sphericals: (batch_size,) or scalar
        """
        if phi_batch.dim() == 1:
            # Single sample
            return self.forward_kinematics_single(phi_batch[0], phi_batch[1], phi_batch[2])
        else:
            # Batch processing
            batch_size = phi_batch.shape[0]
            positions = []
            normals = []
            azimuths = []
            elevations = []
            r_sphericals = []
            
            for i in range(batch_size):
                pos, normal, az, el, r_sph = self.forward_kinematics_single(
                    phi_batch[i, 0], phi_batch[i, 1], phi_batch[i, 2]
                )
                positions.append(pos)
                normals.append(normal)
                azimuths.append(az)
                elevations.append(el)
                r_sphericals.append(r_sph)
            
            return (torch.stack(positions), torch.stack(normals), 
                    torch.stack(azimuths), torch.stack(elevations), torch.stack(r_sphericals))
    
    def action_to_angles(self, actions, angle_range=(0.0, math.pi/2)):
        """
        Map the action space [-1, 1] to joint angle range
        
        Args:
            actions: tensor (..., 3), in range [-1, 1]
            angle_range: (min_angle, max_angle) in radians
            
        Returns:
            angles: tensor (..., 3), joint angles
        """
        actions_clamped = torch.clamp(actions, -1.0, 1.0)
        
        # Linear mapping: [-1, 1] -> [angle_range[0], angle_range[1]]
        min_angle, max_angle = angle_range
        angles = (actions_clamped + 1.0) / 2.0 * (max_angle - min_angle) + min_angle
        
        return angles
    
    def test_kinematics(self):
        """Test if the forward kinematics computations are correct"""
        print("\n🧪 Testing Mozart Kinematics...")
        
        # Test cases - similar to the MATLAB verification
        test_cases = [
            [0.0, 0.0, 0.0],        # Flat
            [0.5, 0.5, 0.5],        # Medium angles (~29°)
            [1.0, 1.0, 1.0],        # Large angles (~57°)
            [0.3, 0.8, 0.2],        # Asymmetric
            [math.pi/6, math.pi/4, math.pi/3],  # 30°, 45°, 60°
        ]
        
        for i, angles in enumerate(test_cases):
            phi1, phi2, phi3 = angles
            pos, normal, azimuth, elevation, r_sph = self.forward_kinematics_single(phi1, phi2, phi3)
            
            print(f"\n  Test {i+1}: φ=[{phi1:.3f}, {phi2:.3f}, {phi3:.3f}] rad")
            print(f"           φ=[{phi1*180/math.pi:.1f}°, {phi2*180/math.pi:.1f}°, {phi3*180/math.pi:.1f}°]")
            print(f"    Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")
            print(f"    Normal:   [{normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f}]")
            print(f"    Spherical: Az={azimuth:.2f}°, El={elevation:.2f}°, R={r_sph:.4f}m")
        
        # Test batch computation
        batch_angles = torch.tensor(test_cases[:3], device=self.device)
        batch_pos, batch_normal, batch_az, batch_el, batch_r = self.forward_kinematics_batch(batch_angles)
        print(f"\n  ✅ Batch processing test passed: {batch_pos.shape}")
        
        print("🎉 Kinematics test completed!")


if __name__ == "__main__":
    # Standalone test for the kinematics module
    kinematics = MozartKinematics(device='cpu')  # Use CPU for testing
    kinematics.test_kinematics()
