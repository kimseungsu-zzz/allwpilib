// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import org.junit.jupiter.api.Test;

/** Public WPILib class-level compatibility harness for the VMX HAL contract. */
class VMXClassLevelIntegrationTest {
  @Test
  void pcmAndRevPhClassesUseTheCompletePublicPath() {
    HAL.initialize(500, 0);

    try (PneumaticsControlModule pcm = new PneumaticsControlModule(10);
        Solenoid pcmSolenoid = pcm.makeSolenoid(2);
        Compressor pcmCompressor = pcm.makeCompressor()) {
      CTREPCMSim sim = new CTREPCMSim(pcm);
      sim.setPressureSwitch(true);
      sim.setCompressorCurrent(12.5);
      sim.setCompressorOn(true);

      pcmSolenoid.set(true);
      assertTrue(pcmSolenoid.get());
      assertTrue(pcm.getPressureSwitch());
      assertEquals(12.5, pcm.getCompressorCurrent());
      assertTrue(pcmCompressor.isEnabled());
      assertFalse(pcm.getCompressorCurrentTooHighFault());
      assertFalse(pcm.getCompressorCurrentTooHighStickyFault());
      assertFalse(pcm.getCompressorShortedFault());
      assertFalse(pcm.getCompressorShortedStickyFault());
      assertFalse(pcm.getCompressorNotConnectedFault());
      assertFalse(pcm.getCompressorNotConnectedStickyFault());
      assertEquals(0, pcm.getSolenoidDisabledList());
      assertFalse(pcm.getSolenoidVoltageFault());
      assertFalse(pcm.getSolenoidVoltageStickyFault());
      pcm.clearAllStickyFaults();
      pcmCompressor.disable();
      assertEquals(CompressorConfigType.Disabled, pcm.getCompressorConfigType());
    }

    try (PneumaticHub ph = new PneumaticHub(11);
        DoubleSolenoid phSolenoid = ph.makeDoubleSolenoid(2, 3);
        Compressor phCompressor = ph.makeCompressor()) {
      REVPHSim sim = new REVPHSim(ph);
      sim.setPressureSwitch(false);
      sim.setCompressorCurrent(8.25);
      sim.setCompressorOn(true);

      phSolenoid.set(DoubleSolenoid.Value.kForward);
      assertEquals(DoubleSolenoid.Value.kForward, phSolenoid.get());
      assertFalse(ph.getPressureSwitch());
      assertEquals(8.25, phCompressor.getCurrent());
      ph.getAnalogVoltage(0);
      ph.getAnalogVoltage(1);
      ph.getPressure(0);
      ph.getPressure(1);
      ph.getVersion();
      ph.getFaults();
      ph.getStickyFaults();
      ph.clearStickyFaults();
      ph.disableCompressor();
      assertEquals(CompressorConfigType.Disabled, ph.getCompressorConfigType());
      ph.enableCompressorHybrid(20, 80);
      assertEquals(CompressorConfigType.Hybrid, ph.getCompressorConfigType());
    }
  }

  @Test
  void pdpAndPdhClassesExposeCurrentAndVoltageReadback() {
    HAL.initialize(500, 0);

    try (PowerDistribution pdp = new PowerDistribution(12, PowerDistribution.ModuleType.kCTRE)) {
      PDPSim sim = new PDPSim(pdp);
      sim.setVoltage(12.4);
      sim.setCurrent(0, 4.5);
      sim.setCurrent(1, 3.25);
      assertEquals(PowerDistribution.ModuleType.kCTRE, pdp.getType());
      assertEquals(12.4, pdp.getVoltage());
      assertEquals(4.5, pdp.getCurrent(0));
      assertEquals(3.25, pdp.getAllCurrents()[1]);
      assertEquals(7.75, pdp.getTotalCurrent());
      assertEquals(96.1, pdp.getTotalPower(), 1e-9);
      assertEquals(0.0, pdp.getTotalEnergy());
      pdp.getVersion();
      pdp.getFaults();
      pdp.getStickyFaults();
      pdp.resetTotalEnergy();
      pdp.clearStickyFaults();
    }

    try (PowerDistribution pdh = new PowerDistribution(13, PowerDistribution.ModuleType.kRev)) {
      PDPSim sim = new PDPSim(pdh);
      sim.setVoltage(13.1);
      sim.setCurrent(0, 6.75);
      sim.setCurrent(23, 1.5);
      assertEquals(24, pdh.getNumChannels());
      assertEquals(13.1, pdh.getVoltage());
      assertEquals(6.75, pdh.getCurrent(0));
      assertEquals(1.5, pdh.getCurrent(23));
      assertEquals(8.25, pdh.getTotalCurrent());
      pdh.getTotalPower();
      pdh.getTotalEnergy();
      pdh.setSwitchableChannel(true);
      pdh.getSwitchableChannel();
      pdh.getVersion();
      pdh.getFaults();
      pdh.getStickyFaults();
      pdh.resetTotalEnergy();
      pdh.clearStickyFaults();
    }
  }

  @Test
  void dioBeamBreakPatternAndPwmServoClassesClose() {
    HAL.initialize(500, 0);

    try (DigitalInput beamBreak = new DigitalInput(18);
        DigitalOutput output = new DigitalOutput(19);
        PWMVictorSPX victor = new PWMVictorSPX(16);
        Servo servo = new Servo(17)) {
      DIOSim inputSim = new DIOSim(beamBreak);
      DIOSim outputSim = new DIOSim(output);
      inputSim.setValue(false);
      output.set(true);
      assertFalse(beamBreak.get());
      assertTrue(output.get());
      assertTrue(outputSim.getValue());

      victor.set(0.4);
      servo.setAngle(90);
      assertEquals(0.4, new PWMSim(victor.getChannel()).getSpeed(), 1.0 / 2000.0);
      assertEquals(90, servo.getAngle(), 1.0 / 2000.0);
    }
  }
}
