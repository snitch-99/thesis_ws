import numpy as np
import scipy.optimize

def compute_pca_frame(points):
    """
    Computes the Principal Component Analysis (PCA) frame.
    Z-axis = Normal (Direction of least variance - actually for a bump, 
             the Z axis is usually the direction of height, so we might want 
             Direction of LEAST variance to be Z for a FLAT patch, 
             but for a BUMP, the 'Height' is significant?)
    
    Standard Surface Normal estimation uses Least Variance as Normal.
    So local Z = Least Variance Vector.
    """
    center = np.mean(points, axis=0)
    centered_pts = points - center
    cov = np.cov(centered_pts.T)
    eig_vals, eig_vecs = np.linalg.eigh(cov)
    
    # eigh returns ascending order.
    # vec[:, 0] is eigenvector for smallest eigenvalue (Normal direction for surface)
    # vec[:, 1] and vec[:, 2] are tangent plane directions
    
    # We want local Z to be the "Height" direction (Normal).
    z_axis = eig_vecs[:, 0]
    y_axis = eig_vecs[:, 1]
    x_axis = eig_vecs[:, 2]
    
    # Orientation check (consistent normal?)
    # We can't easily know "outward" without origin, but we can fix later.
    
    R = np.column_stack((x_axis, y_axis, z_axis))
    return R, center

def fit_superquadric(points):
    """
    Fits a Superquadric to the given points (centered).
    Returns parameters: [a1, a2, a3, e1, e2]
    """
    # Initial Guess
    extent = np.max(np.abs(points), axis=0)
    a1_init, a2_init, a3_init = extent[0], extent[1], extent[2]
    
    # Avoid zero
    if a1_init < 1e-3: a1_init = 1.0 
    if a2_init < 1e-3: a2_init = 1.0
    if a3_init < 1e-3: a3_init = 1.0
    
    x0 = [a1_init, a2_init, a3_init, 1.0, 1.0] # a1, a2, a3, e1, e2
    
    # Bounds: a > 0, 0.1 < e < 3.0
    lower_bound = [1e-4, 1e-4, 1e-4, 0.1, 0.1]
    upper_bound = [np.inf, np.inf, np.inf, 3.0, 3.0]
    
    def implicit_sq(params, p):
        a1, a2, a3, e1, e2 = params
        x = p[:, 0]
        y = p[:, 1]
        z = p[:, 2]
        
        # ((|x/a1|^(2/e2) + |y/a2|^(2/e2))^(e2/e1) + |z/a3|^(2/e1)) - 1 = 0
        
        t1_inner = (np.abs(x/a1)**(2/e2)) + (np.abs(y/a2)**(2/e2))
        t1 = t1_inner**(e2/e1)
        t2 = np.abs(z/a3)**(2/e1)
        
        return t1 + t2 - 1.0

    def residuals(params, p):
        # We want to minimize (F(x,y,z) - 0)^2
        # But for least_squares we return the vector of residuals
        # Implicit function value IS the error (if = 0, on surface)
        # Note: Implicit function grows fast, improved distance approx is better efficiently
        # but for simple fitting specific patches, implicit might suffice.
        # Better: use radial distance approximation like in EMS but simplified.
        # For now, explicit algebraic distance (implicit eq) is a standard start.
        return implicit_sq(params, p)

    res = scipy.optimize.least_squares(
        residuals, 
        x0, 
        args=(points,),
        bounds=(lower_bound, upper_bound),
        method='trf',
        ftol=1e-4,
        xtol=1e-4
    )
    
    return res.x

def generate_superquadric_points(params, resolution=50):
    """
    Generates points for visualization given [a1, a2, a3, e1, e2].
    """
    a1, a2, a3, e1, e2 = params
    
    eta = np.linspace(-np.pi/2, np.pi/2, resolution)
    omega = np.linspace(-np.pi, np.pi, resolution)
    ETA, OMEGA = np.meshgrid(eta, omega)
    
    def signed_pow(val, p):
        return np.sign(val) * (np.abs(val)**p)
    
    ce = np.cos(ETA)
    se = np.sin(ETA)
    co = np.cos(OMEGA)
    so = np.sin(OMEGA)
    
    x = a1 * signed_pow(ce, e1) * signed_pow(co, e2)
    y = a2 * signed_pow(ce, e1) * signed_pow(so, e2)
    z = a3 * signed_pow(se, e1)
    
    pts = np.column_stack((x.flatten(), y.flatten(), z.flatten()))
    return pts

def fit_paraboloid(points):
    """
    Fits a paraboloid to the given points (centered and aligned).
    Paraboloid equation: z = a*x^2 + b*y^2 + c*x*y + d*x + e*y + f
    Returns parameters: [a, b, c, d, e, f]
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # Build design matrix for least squares
    # [x^2, y^2, xy, x, y, 1]
    A = np.column_stack([
        x**2,
        y**2,
        x*y,
        x,
        y,
        np.ones_like(x)
    ])
    
    # Solve least squares: A @ params = z
    params, residuals, rank, s = np.linalg.lstsq(A, z, rcond=None)
    
    return params

def generate_paraboloid_points(params, x_range, y_range, resolution=50):
    """
    Generates points for visualization given paraboloid parameters.
    params: [a, b, c, d, e, f]
    x_range: (x_min, x_max)
    y_range: (y_min, y_max)
    """
    a, b, c, d, e, f = params
    
    x = np.linspace(x_range[0], x_range[1], resolution)
    y = np.linspace(y_range[0], y_range[1], resolution)
    X, Y = np.meshgrid(x, y)
    
    Z = a*X**2 + b*Y**2 + c*X*Y + d*X + e*Y + f
    
    pts = np.column_stack((X.flatten(), Y.flatten(), Z.flatten()))
    return pts

def fit_paraboloid_em(points, distance_threshold=0.05, w_o_prior=0.3, max_iters=20, verbose=True):
    """
    Fits a paraboloid using EM algorithm with inlier/outlier classification.
    
    Args:
        points: Nx3 array of points (centered and aligned)
        distance_threshold: Distance threshold for inlier classification (meters)
        w_o_prior: Prior probability of a point being an outlier
        max_iters: Maximum number of EM iterations
        verbose: Print iteration details
    
    Returns:
        params: [a, b, c, d, e, f] paraboloid parameters
        inlier_mask: Boolean mask of inlier points
        final_inlier_count: Number of inliers in final iteration
    """
    if verbose:
        print(f"\n  EM Paraboloid Fitting:")
        print(f"    Total points: {len(points)}")
        print(f"    Distance threshold: {distance_threshold}m")
        print(f"    Outlier prior: {w_o_prior}")
    
    # Initial fit using all points
    params = fit_paraboloid(points)
    
    for iteration in range(max_iters):
        # E-Step: Classify inliers/outliers
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        a, b, c, d, e, f = params
        
        # Predicted z values
        z_pred = a*x**2 + b*y**2 + c*x*y + d*x + e*y + f
        
        # Residuals (distance from paraboloid)
        residuals = np.abs(z - z_pred)
        
        # Inlier probability (simplified: hard threshold)
        # Could use Gaussian likelihood for soft assignment
        inlier_mask = residuals <= distance_threshold
        num_inliers = np.sum(inlier_mask)
        
        if verbose:
            print(f"    Iter {iteration+1}: {num_inliers}/{len(points)} inliers, "
                  f"mean residual: {np.mean(residuals):.4f}m")
        
        # Check convergence (if inlier count stabilizes)
        if iteration > 0 and num_inliers == prev_inlier_count:
            if verbose:
                print(f"    Converged at iteration {iteration+1}")
            break
        
        prev_inlier_count = num_inliers
        
        # M-Step: Re-fit using only inliers
        if num_inliers < 6:
            if verbose:
                print(f"    WARNING: Too few inliers ({num_inliers}), stopping.")
            break
        
        inlier_points = points[inlier_mask]
        params = fit_paraboloid(inlier_points)
    
    # Final statistics
    if verbose:
        print(f"    Final inliers: {num_inliers}/{len(points)} ({100*num_inliers/len(points):.1f}%)")
        print(f"    Final params: a={params[0]:.4f}, b={params[1]:.4f}, c={params[2]:.4f}")
    
    return params, inlier_mask, num_inliers
