
static void step(void){tick++;TIM_HandleTypeDef t={TIM6};HAL_TIM_PeriodElapsedCallback(&t);}
static void steps(unsigned n){for(unsigned i=0;i<n;i++)step();}
static void command(char*s){HandleUsbLine(s,tick);}
static void zero(void){for(int i=0;i<3;i++)assert(out[i]==0);}
int main(void){

  const float gains[3][2][3]={{{6.5f,26.0f,0.0f},{7.0f,15.0f,0.0f}},{{6.0f,54.0f,0.0f},{8.4f,40.0f,0.0f}},{{6.0f,50.0f,0.0f},{6.5f,40.0f,0.0f}}};
  const float angle_gains[3][2][3]={{{9.0f,2.0f,0.0f},{9.0f,2.0f,0.0f}},{{9.0f,2.0f,0.0f},{9.0f,2.0f,0.0f}},{{9.0f,3.0f,0.0f},{9.0f,2.0f,0.0f}}};
  const float expected_targets[3]={-70.0f,-30.0f,-30.0f};
  const float expected_min[3]={50.0f,50.0f,40.0f};
  const float expected_max[3]={100.0f,100.0f,150.0f};
  assert(k_g==1.6f && angle_settle_error_deg==0.7f && angle_settle_time_ms==10);
  assert(speed_period_ms==5 && filter_period_ms==5 && control_mode==1);
  InitializePids();
  for(int i=0;i<3;i++){
    assert(angle_target[i]==expected_targets[i] && angle_max_speed[i]==expected_max[i] && angle_min_speed[i]==expected_min[i] && motor_friction_raw[i]==0 && speed_target[i]==0);
    for(int d=0;d<2;d++){
      assert(speed_pid_gains[i][d].kp==gains[i][d][0] && speed_pid_gains[i][d].ki==gains[i][d][1] && speed_pid_gains[i][d].kd==gains[i][d][2]);
      assert(angle_pid_gains[i][d].kp==angle_gains[i][d][0] && angle_pid_gains[i][d].ki==angle_gains[i][d][1] && angle_pid_gains[i][d].kd==angle_gains[i][d][2]);
    }
    assert(angle_pid[i].kp==angle_gains[i][0][0] && angle_pid[i].ki==angle_gains[i][0][1] && angle_pid[i].kd==angle_gains[i][0][2]);
    assert(angle_pid[i].max_output==expected_max[i]);angles[i]=12.77f;
  }
  assert(motor_output_scale[0]==1 && motor_output_scale[1]==.9f && motor_output_scale[2]==1.1f);
  CAN_SetSpeedFilterPeriod(5);
  steps(10);zero(); // JSON target presets cannot auto-start.
  // The following regression scenarios explicitly configure their historical test fixture.
  command("H,500,20");
  for(int i=1;i<=3;i++){
    char line[64];snprintf(line,sizeof(line),"J,%d,-1,3000,0,0,100000,0",i);command(line);
    snprintf(line,sizeof(line),"J,%d,1,3000,0,0,100000,0",i);command(line);
  }


  // The new bound only affects nonzero outer-loop speeds outside the arrival band.
  command("J,1,-1,1000,0,0,10000,3000");command("J,1,1,1000,0,0,10000,3000");
  assert(control_mode==1 && angle_min_speed[0]==3 && angle_min_speed[1]==0);
  command("A,14770,12770,12770");steps(5);assert(speed_target[0]==3);
  angles[0]=16.77f;steps(5);assert(speed_target[0]==-3);
  angles[0]=-50;steps(5);assert(speed_target[0]==10); // Upper bound remains dominant.
  angles[0]=14.52f;steps(5);assert(angle_settle_state[0]==1 && speed_target[0]>0 && speed_target[0]<.3f);
  steps(21);assert(angle_settle_state[0]==2 && speed_target[0]==0);
  command("S");step();zero();
  command("J,1,1,9000,0,0,10000,11000");assert(angle_min_speed[0]==3 && angle_pid_gains[0][1].kp==1);
  command("J,1,1,9000,0,0,10000,-1");assert(angle_pid_gains[0][1].kp==1);
  command("J,1,1,9000,0,0,10000,2000,9");assert(angle_pid_gains[0][1].kp==1);
  command("J,1,-1,0,0,0,10000,3000");command("J,1,1,0,0,0,10000,3000");
  angles[0]=10;command("A,14770,12770,12770");steps(5);assert(speed_target[0]==0); // Zero PID stays disabled.
  command("S");step();zero();
  command("J,1,1,3000,0,0,100000");assert(angle_min_speed[0]==0 && control_mode==0); // Legacy six-parameter J.
  command("J,1,-1,3000,0,0,100000,0");
  for(int i=0;i<3;i++)angles[i]=12.77f;
  command("A,-60000,-60000,-60000");steps(5);assert(control_mode==6 && out[0]<0);
  // Crossing the removed numeric boundaries must not lock or disable the loop.
  angles[0]=-.1f;step();angles[0]=.1f;step();assert(control_mode==6);
  angles[1]=-80.1f;step();assert(control_mode==6);
  command("A,12770,12770,12770");assert(angle_target[0]==12.77f && control_mode==6);
  for(int i=0;i<3;i++)angles[i]=12.77f;
  steps(21);for(int i=0;i<3;i++)assert(angle_settle_state[i]==2 && speed_target[i]==0);
  command("S");step();assert(control_mode==0);zero();
  command("A,0,0,0");steps(301);assert(control_mode==0);zero(); // Host heartbeat loss.
  command("A,0,0,0");fresh_mask=3;step();assert(control_mode==0);zero(); // CAN loss.
  fresh_mask=7;command("F,1,0,16384");motor_output_scale[0]=5;step();assert(out[0]==16384);
  command("S");step();zero();
  command("J,1,-1,3000,0,0,100000,2000");command("J,2,-1,3000,0,0,100000,3000");command("J,3,-1,3000,0,0,100000,4000");
  assert(control_mode==0);send_telemetry(tick+5);
  unsigned fields=1;for(char*p=wire;*p;p++)if(*p==',')fields++;
  assert(fields==108 && strncmp(wire,"V11,",4)==0 && strlen(wire)<1280);
  const char *tail=",2000,3000,4000\r\n";
  assert(strcmp(wire+strlen(wire)-strlen(tail),tail)==0);
  puts("PASS: actual V11 min-speed: +/- floor, upper cap, arrival confirmation without floor, settled zero, zero-PID zero, invalid/legacy J, shared bound and actual append-only fields; defaults, no auto-start, 0/-80 crossing and 12.77 home, stable arrival, S, heartbeat/CAN loss, final current clipping and 108-field telemetry.");
}
