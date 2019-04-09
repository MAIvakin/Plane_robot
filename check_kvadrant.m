function check_kvadrant = check_kvadrant(X_c, Y_c,psi_c)
if Y_c <0 && X_c>0
    psi_c = 180 - psi_c;
end
   if Y_c <0 && X_c < 0
   psi_c = 180 + psi_c; 
   end
   if  Y_c >0 && X_c < 0
    psi_c = 360 - psi_c; 
   end
   assignin('base','psi_c',psi_c)
end