GENERALIZED PREDICTIVE CONTROL (GPC)
====================================

Generalized predictive controle tests as proposed by [1] adn [2].

* teste_transfer_funcrion: generate control law by quadratic programming or via GPC expression, test model mismach, create G, Gl and F matrix using long polynomial division, see the behavior of optimal estimation of model with uncertainty and noise. 
* test_gpg_concepts: gerate the matrixes using polynomial division [3] and generate control law using the analitical solution of MPC without quadratic programming.  
* poly_long_div_v2: polynomial division to generate prediction matrixes of GPC
* gpc_siso_example: GPC example without considering the deadtime in the model (a little dead time make the solution unstable)  
* gpc_siso_delay: GPC example considering the dead time of process in the model.  
* gpc_matlab_example: Using matlab toolbox to solve the GPC problem formulation
* carima_model_forced_free_response: example to see the behavior of optimal prediction of carima model used in GPC under uncertainty and noise


[1] Carlos Bordons and E. F. Camacho, Model Predictive Control in the Process Industry  
[2] Carlos Bordons and E. F. Camacho, Model Predictive Control  
[3] Plinio Castrucci, Roberto Moura Sales; Controle Digital
