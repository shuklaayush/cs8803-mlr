 m = csvread ('C:\Users\bghader\Desktop\cs8803-mlr-master\cs8803-mlr-master\hw3\data\input1.csv');
 n = csvread ('C:\Users\bghader\Desktop\cs8803-mlr-master\cs8803-mlr-master\hw3\data\output1.csv');
 t_in = m(:,1) ; 
 t_out = n(:,1); 
 in  = m(:,3)  ; 
 out = n(:,3)  ;
 plot(t_in,in) ; 
 hold 
 plot(t_out,out); % for visualization  
 %%
out= interp1 (t_out,out,t_in) ;  
plot (t_in,in) ; 
hold 
plot (t_in,out); 

find (isnan(out)==1)
find (isnan(in)==1)
out  = out(5:1353) ;
in =  in(5:1353) ;  
t_in =  t_in(5:1353) ;
%% Validation Data
 m_v = csvread ('C:\Users\bghader\Desktop\cs8803-mlr-master\cs8803-mlr-master\hw3\data\input2.csv');
 n_v = csvread ('C:\Users\bghader\Desktop\cs8803-mlr-master\cs8803-mlr-master\hw3\data\output2.csv');
 t_in_v = m_v(:,1) ; 
 t_out_v = n_v(:,1); 
 in_v  = m_v(:,3)  ; 
 out_v = n_v(:,3)  ;
 plot(t_in_v,in_v) ; 
 hold 
 plot(t_out_v,out_v); % for visualization  
 out_v= interp1 (t_out_v,out_v,t_in_v) ;
 out_v = out_v(3:2159)
 in_v = in_v(3:2159)
