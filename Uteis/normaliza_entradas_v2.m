function uk = normaliza_entradas_v2(u)
% normaliza as entradas
freq_n = normalizar_dado_BCS_v2(u(1),'frequencia_BCSS');
Pmon_n = normalizar_dado_BCS_v2(u(2),'pressao_montante_alvo');
% normaliza os regressores
Psuc_n = normalizar_dado_BCS_v2(u(3),    'pressao_succao_BCSS');
Pche_n = normalizar_dado_BCS_v2(u(4),    'pressao_chegada');
Pdifer_n = normalizar_dado_BCS_v2(u(5),  'pressao_diferencial_BCSS');
Pdesc_n = normalizar_dado_BCS_v2(u(6),   'pressao_descarga_BCSS');
Tmotor_n = normalizar_dado_BCS_v2(u(7),  'temperatura_motor_BCSS');
Ctorque_n = normalizar_dado_BCS_v2(u(8), 'corrente_torque_BCSS');
Ctotal_n = normalizar_dado_BCS_v2(u(9),  'corrente_total_BCSS');
Tsuc_n = normalizar_dado_BCS_v2(u(10),   'temperatura_succao_BCSS');
Vib_n = normalizar_dado_BCS_v2(u(11),    'vibracao_BCSS');
Tche_n = normalizar_dado_BCS_v2(u(12),   'temperatura_chegada');
uk = [freq_n;Pmon_n;Psuc_n;Pche_n;Pdifer_n;Pdesc_n;Tmotor_n;Ctorque_n;Ctotal_n;Tsuc_n;Vib_n;Tche_n];
end