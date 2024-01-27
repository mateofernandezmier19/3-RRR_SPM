function Measure_Orientation(ser,~)
value=readline(ser);
vsplit=strsplit(value,',');
theta=str2double(vsplit(1));
psi=str2double(vsplit(2));
phi=str2double(vsplit(3));
ser.UserData=[phi,theta,psi];
end