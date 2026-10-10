% cvt_probe.m — CV Toolbox 关键函数可用性实测
ok_orb = 0; ok_f = 0; ok_e = 0; ok_decomp = 0; ok_homo = 0; ok_spm = 0;
try
    det = orb('FeaturesPerImage', 300);
    img = imread('/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town05Data_IMU_Fusion/0001.png');
    pts = extractFeatures(img, det);
    ok_orb = numel(pts);
catch e
    ok_orb = -1; m1 = e.message;
end
try
    [F, inl] = estimateFundamentalMatrix(rand(50,2), rand(50,2));
    ok_f = 1;
catch e
    ok_f = -1; m2 = e.message;
end
try
    E = estimateEssentialMatrix([1 1;2 2;3 3;4 4], [1 2;2 3;3 4;4 5], 1, 1);
    ok_e = 1;
catch e
    ok_e = -1; m3 = e.message;
end
try
    [R, t] = decomposeMotion(eye(3), zeros(3,1));
    ok_decomp = 1;
catch e
    ok_decomp = -1; m4 = e.message;
end
try
    [H, inl] = estimateHomography(rand(20,2), rand(20,2));
    ok_homo = 1;
catch e
    ok_homo = -1; m5 = e.message;
end
try
    out = solvepnp(rand(10,3), rand(10,2), eye(3), zeros(3,1), []);
    ok_spm = 1;
catch e
    ok_spm = -1; m6 = e.message;
end
fprintf('RESULT ORB=%d F=%d E=%d DECOMP=%d HOMO=%d PNP=%d\n', ...
    ok_orb, ok_f, ok_e, ok_decomp, ok_homo, ok_spm);
if ok_orb < 0,  fprintf('ORB_ERR: %s\n', m1); end
if ok_f < 0,    fprintf('F_ERR: %s\n', m2); end
if ok_e < 0,    fprintf('E_ERR: %s\n', m3); end
if ok_decomp < 0, fprintf('D_ERR: %s\n', m4); end
if ok_homo < 0, fprintf('H_ERR: %s\n', m5); end
if ok_spm < 0,  fprintf('PNP_ERR: %s\n', m6); end
