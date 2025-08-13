mkdir -p testcases/SCR000001/case1 testcases/SCR000001/case2

cat > testcases/SCR000001/case1/case1.csv <<'EOF'
time,ignition_switch
0,0
1,0
2,1
3,1
4,1
5,0
EOF

cat > testcases/SCR000001/case1/golden.csv <<'EOF'
time,engine_state
0,0
1,0
2,1
3,1
4,1
5,0
EOF

cat > testcases/SCR000001/case2/case2.csv <<'EOF'
time,ignition_switch
0,1
1,1
2,0
3,0
4,1
5,1
6,0
7,1
8,0
9,1
EOF

cat > testcases/SCR000001/case2/golden.csv <<'EOF'
time,engine_state
0,1
1,1
2,0
3,0
4,1
5,1
6,0
7,1
8,0
9,1
EOF
