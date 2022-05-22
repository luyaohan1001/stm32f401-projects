
/**
  * @brief  Resume callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
  USBD_LL_Resume((USBD_Handle_t*)hpcd->pData);
}



/**
  * @brief  USBD_DeInit
  *         Re-Initialize the device library
  * @param  pdev: device instance
  * @retval status: status
  */
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev)
{
  USBD_StatusTypeDef ret;

  /* Disconnect the USB Device */
  USBD_LL_Stop(pdev);

  /* Set Default State */
  pdev->dev_state = USBD_STATE_DEFAULT;

  /* Free Class Resources */
  if (pdev->pClass != NULL)
  {
    pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
  }

  pdev->pUserData[0] = NULL;

  /* Free Device descriptors resources */
  pdev->pDesc = NULL;
  pdev->pConfDesc = NULL;

  /* DeInitialize low level driver */
  ret = USBD_LL_DeInit(pdev);

  return ret;
}


/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete((USBD_Handle_t*)hpcd->pData, epnum);
}

