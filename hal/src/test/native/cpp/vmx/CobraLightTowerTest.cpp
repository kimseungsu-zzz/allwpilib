// Copyright (c) 2026 WPILib contributors.

#include <gtest/gtest.h>

#include <cstdint>

#include "../../../../main/native/vmx/DigitalChannelRegistry.h"
#include "../../../../main/native/vmx/VMXConstants.h"
#include "studica/Cobra.h"
#include "studica/LightTower.h"

TEST(StudicaCobraVendorTest, AbiAndChannelContractAreFixedWidth) {
  EXPECT_EQ(sizeof(StudicaCobraHandle), sizeof(uint32_t));
  EXPECT_EQ(STUDICA_COBRA_CHANNEL_COUNT, 4U);
  StudicaCobraHandle handle = 0;
  EXPECT_EQ(StudicaCobra_Create(0.0, &handle),
            STUDICA_COBRA_INVALID_ARGUMENT);
  EXPECT_EQ(StudicaCobra_Create(5.5, &handle),
            STUDICA_COBRA_INVALID_ARGUMENT);
}

TEST(StudicaCobraVendorTest, HostBindingExposesStableMetadataWithoutFakingData) {
  StudicaCobraHandle handle = 0;
  ASSERT_EQ(StudicaCobra_Create(5.0, &handle), STUDICA_COBRA_OK);
  uint8_t channels = 0;
  double reference = 0.0;
  EXPECT_EQ(StudicaCobra_GetChannelCount(handle, &channels), STUDICA_COBRA_OK);
  EXPECT_EQ(channels, 4U);
  EXPECT_EQ(StudicaCobra_GetReferenceVoltage(handle, &reference),
            STUDICA_COBRA_OK);
  EXPECT_DOUBLE_EQ(reference, 5.0);
  int32_t raw = 0;
  EXPECT_EQ(StudicaCobra_GetRaw(handle, 4, &raw),
            STUDICA_COBRA_INVALID_ARGUMENT);
  EXPECT_EQ(StudicaCobra_GetRaw(handle, 0, &raw), STUDICA_COBRA_UNAVAILABLE);
  StudicaCobra_Destroy(handle);
  EXPECT_EQ(StudicaCobra_GetRaw(handle, 0, &raw),
            STUDICA_COBRA_NOT_INITIALIZED);
}

TEST(StudicaLightTowerVendorTest, ValidatesDistinctPhysicalOutputs) {
  StudicaLightTowerHandle handle = 0;
  EXPECT_EQ(StudicaLightTower_Create(0, 0, 2, 3, 4, &handle),
            STUDICA_LIGHT_TOWER_INVALID_ARGUMENT);
  EXPECT_EQ(StudicaLightTower_Create(0, 1, 2, 3, 4, &handle),
            STUDICA_LIGHT_TOWER_OK);
  EXPECT_NE(handle, 0U);
  EXPECT_EQ(StudicaLightTower_SetSolid(handle),
            STUDICA_LIGHT_TOWER_UNAVAILABLE);
  StudicaLightTower_Destroy(handle);
}

TEST(StudicaVendorRegistryTest, CobraSharesOnlyThePhysicalI2CBus) {
  hal::vmx::DigitalChannelRegistry registry;
  EXPECT_TRUE(registry
                  .Reserve(32, hal::vmx::DigitalChannelOwner::kDIO, "DIO")
                  .reserved);
  EXPECT_FALSE(registry
                   .ReserveShared(32, hal::vmx::DigitalChannelOwner::kCobra,
                                  "Cobra", hal::vmx::DigitalChannelOwner::kI2C)
                   .reserved);

  hal::vmx::DigitalChannelRegistry shared;
  EXPECT_TRUE(shared
                  .ReserveShared(32, hal::vmx::DigitalChannelOwner::kI2C,
                                 "I2C", hal::vmx::DigitalChannelOwner::kCobra)
                  .reserved);
  EXPECT_TRUE(shared
                  .ReserveShared(32, hal::vmx::DigitalChannelOwner::kCobra,
                                 "Cobra", hal::vmx::DigitalChannelOwner::kI2C)
                  .reserved);
  shared.ReleaseShared(32, hal::vmx::DigitalChannelOwner::kCobra);
  EXPECT_FALSE(shared
                   .Reserve(32, hal::vmx::DigitalChannelOwner::kDIO, "DIO")
                   .reserved);
  shared.ReleaseShared(32, hal::vmx::DigitalChannelOwner::kI2C);
  EXPECT_TRUE(shared
                  .Reserve(32, hal::vmx::DigitalChannelOwner::kDIO, "DIO")
                  .reserved);
}

TEST(StudicaVendorFacadeTest, CppFacadesUseTheSameCAbi) {
  studica::Cobra cobra;
  EXPECT_EQ(cobra.GetChannelCount(), 4U);
  EXPECT_DOUBLE_EQ(cobra.GetReferenceVoltage(), 5.0);
  EXPECT_EQ(cobra.GetRaw(0), -1);
  EXPECT_EQ(cobra.GetLastStatus(), STUDICA_COBRA_UNAVAILABLE);

  studica::LightTower tower{1, 2, 3, 4, 5};
  EXPECT_FALSE(tower.SetSolid());
  EXPECT_EQ(tower.GetLastStatus(), STUDICA_LIGHT_TOWER_UNAVAILABLE);
}
