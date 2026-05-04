function q = eul2quat(phi, theta, psi)
    % phi: Roll (rad), theta: Pitch (rad), psi: Yaw (rad)
    % Retorna quaternião na forma [w; x; y; z]
    
    cp = cos(phi/2); sp = sin(phi/2);
    ct = cos(theta/2); st = sin(theta/2);
    cs = cos(psi/2); ss = sin(psi/2);
    
    w = cp*ct*cs + sp*st*ss;
    x = sp*ct*cs - cp*st*ss;
    y = cp*st*cs + sp*ct*ss;
    z = cp*ct*ss - sp*st*cs;
    
    q = [w; x; y; z];
end