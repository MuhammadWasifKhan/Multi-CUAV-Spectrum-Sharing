function z=ternary_shadow_z(Shadow,sc,band)
if band==1,z=Shadow.a2g_pu_z(sc);else,z=Shadow.a2g_wu_z(sc);end
end
