function yk_1 = desnormaliza_predicoes_v2(yk_aux)
% normaliza as entradas
% desnormaliza as saídas
Psuc_desn = desnormalizar_dado_BCS_v2(yk_aux(1),    'pressao_succao_BCSS');
Pche_desn = desnormalizar_dado_BCS_v2(yk_aux(2),    'pressao_chegada');
Pdifer_desn = desnormalizar_dado_BCS_v2(yk_aux(3),  'pressao_diferencial_BCSS');
Pdesc_desn = desnormalizar_dado_BCS_v2(yk_aux(4),   'pressao_descarga_BCSS');
Tmotor_desn = desnormalizar_dado_BCS_v2(yk_aux(5),  'temperatura_motor_BCSS');
Ctorque_desn = desnormalizar_dado_BCS_v2(yk_aux(6), 'corrente_torque_BCSS');
Ctotal_desn = desnormalizar_dado_BCS_v2(yk_aux(7),  'corrente_total_BCSS');
Tsuc_desn = desnormalizar_dado_BCS_v2(yk_aux(8),    'temperatura_succao_BCSS');
Vib_desn = desnormalizar_dado_BCS_v2(yk_aux(9),     'vibracao_BCSS');
Tche_desn = desnormalizar_dado_BCS_v2(yk_aux(10),   'temperatura_chegada');
yk_1 = [Psuc_desn;Pche_desn;Pdifer_desn;Pdesc_desn;Tmotor_desn;Ctorque_desn;Ctotal_desn;Tsuc_desn;Vib_desn;Tche_desn];
end