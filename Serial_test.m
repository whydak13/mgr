delete(instrfindall);
serial4 = serial('COM6', 'BaudRate', 115200,'timeout',3);
serial4.InputBufferSize=1000;
fopen(serial4)

 while(1)
fwrite(serial4,'1234567890','uchar','async')
% pause(0.1)
% [A2,count] = fscanf(serial4)
X_F_STM = fread(serial4,7,'float')



[ f ] =get_f( Np, [0 0], F, X_F_STM(1:6), Phi );
DeltaU=QPhild2(Phi_Phi,f,A_cons,B_cons);

 X_F_STM(7)
 DeltaU(1)
 end
% strtrim(A2)
% X_F_STM=X_F_STM./1000000000;
fclose(serial4)

