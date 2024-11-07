// 팀 이름 display LCD_ShowString(x,y,string,string_color,background_color)
LCD_ShowString(20,50, "THU_TEAM00", GRAY, WHITE);


void AdcInit(void) {
    ADC_InitTypeDef ADC_InitStructure;

    ADC_InitStructure.ADC_Mode = ;
    ADC_InitStructure.ADC_ScanConvMode = ;
    ADC_InitStructure.ADC_ContinuousConvMode = ;
    ADC_InitStructure.ADC_ExternalTrigConv = ;
    ADC_InitStructure.ADC_DataAlign = ;
    ADC_InitStructure.ADC_NbrOfChannel = ;
    ADC_Init(???, &ADC_InitStructure);

    ADC_RegularChannelConfig(???, ???, ???, ???);
    ADC_ITConfig(???, ADC_IT_EOC, ENABLE);
    ADC_Cmd(???, ENABLE);
    ADC_ResetCalibration(???);

    while(ADC_GetResetCalibrationStatus(???));

    ADC_StartCalibration(???);

    while(ADC_GetCalibrationStatus(???));

    ADC_SoftwareStartConvCmd(???, ENABLE);
}