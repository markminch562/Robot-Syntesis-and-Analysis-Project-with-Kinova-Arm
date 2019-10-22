% Mark Minch, Michal Pham, 
q1=q_2_rad(180, 180, 180, 180, 180, 180, 180 ,1);
disp("end effector postion one is")
disp(q1)

q2=q_2_rad(0, 100, 280, 65, 10, 210, 210, 2);
disp("end effector position two is")
disp(q2)

q3=q_2_rad(273, 183, 390, 49, 258, 288, 288, 3);
disp("end effector position three is")
disp(q3)

q4=q_2_rad(323, 210 ,166, 88, 190, 233, 233, 4);
disp("end effector position four is")
disp(q4)

q5=q_2_rad(283, 163, 0, 44, 265, 258, 258, 5);
disp("end effector position five is")
disp(q5)


function qs =q_2_rad(q1,q2,q3,q4,q5,q6,q7, pos)
q1=deg2rad(q1);
q2=deg2rad(q2);
q3=deg2rad(q3);
q4=deg2rad(q4);
q5=deg2rad(q5);
q6=deg2rad(q6);
q7=deg2rad(q7);
% convert deg to radians for each test set we send it
D1= .2755;
D2= .2050;
D3= .2050;
D4= .2073;
D5 = .1038;
D6 = .1038;
D7 = .1600;
e2 = .0098;
% calls function d_h to create all the d_h frames for each q angle 
h0 = d_h(0, 0, 0, 0);
h1 = d_h(0, pi/2, D1, q1);
h2 = d_h(0, pi/2, 0, q2);
h3 = d_h(0, pi/2, (D2+D3), q3);
h4 = d_h(0, pi/2, e2, q4);
h5 = d_h(0, pi/2, (D4+D5), q5);
h6 = d_h(0, pi/2, 0, q6);
h7 = d_h(0, pi, (D6+D7), q7);
%multiply all d_h matrix together to get final positon of end effector
%relative to base, qs passes back final d_h matrix

qs = h0*h1*h2*h3*h4*h5*h6*h7;
k=strcat('the jacobian for arm postion_', num2str(5));
k=strcat(k, '_is');
disp(k)
% call function to find jacobians
jzz = jz(h0,h1, h2, h3, h4, h5, h6, h7, qs);
disp(jzz)

end

% preforms matrix math to find each dh 0_n so the final jacobian can be
% found
function jacobs = jz(j0,j1, j2, j3, j4, j5,  j6, j7, j8)

jz0 = d_n(j0, j8);

j0_1 = j0*j1;
jz1 = d_n(j0_1, j8);

j0_2 = j0_1*j2;
jz2 = d_n(j0_2, j8);

j0_3 = j0_2*j3;
jz3 = d_n(j0_3, j8);

j0_4 = j0_3*j4;
jz4 = d_n(j0_4, j8);

j0_5 = j0_4*j5;
jz5 = d_n(j0_5, j8);

j0_6 = j0_5*j6;
jz6 = d_n(j0_6, j8);

jacobs = [jz0 jz1 jz2 jz3 jz4 jz5 jz6];
end

%performs the math to find the jacobians
function Js = d_n(hn, hn1)
hn_r= hn(1:3,1:3);
hn_n = hn(1:3, 4);
hn_f = hn1(1:3, 4);

hn_r = hn_r*[0; 0; 1];
hn_cross = cross(hn_r, hn_f-hn_n);
Js = [hn_cross; hn_r];

end
function H = d_h(ai, alfi, di,thei)
H = [cos(thei) -sin(thei)*cos(alfi) sin(thei)*sin(alfi) ai*cos(thei);
    sin(thei) cos(thei)*cos(alfi) -cos(thei)*sin(alfi) ai*sin(thei);
    0 sin(alfi) cos(alfi) di;
    0 0 0 1];
end

