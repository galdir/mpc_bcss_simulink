function res = predict_LSTM_Casadi(modelo_casadi, x_input)

    res = modelo_casadi.casadi_func(x_input, modelo_casadi.h0_input, modelo_casadi.c0_input);
    res=res(end);

end