# Interface MX25R (NOR Flash)

Cette librairie permet de s'interfacer avec une flash Macronix MX25R (ou compatible)

## Importation du module

L'interface est disponible dans le manifest CISTEME, accessible via [ce lien](https://github.com/QPCisteme/zephyr_cisteme-manifest).

## Utilisation
Une fois le module importé, déclarer la flash dans le DeviceTree à l'aide de la compatibilité *cisteme,mx25r*

<u>Exemple nRF52840DK :</u>

```
&spi2 {
    compatible = "nordic,nrf-spim";
    status = "okay";
    pinctrl-0 = <&spi2_default>;
    pinctrl-1 = <&spi2_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;
    mx25r: mx25r@0 {
        status="okay";
        compatible = "cisteme,mx25r";
        reg = <0>;
        spi-max-frequency = <8000000>;
        label = "MX25R64";
    };
};
```
Dans le code C il suffit ensuite d'appeler la flash à l'aide d'une fonction DEVICE_DT_GET comme par exemple :
```c
static const struct device *flash = DEVICE_DT_GET_ANY(cisteme_mx25r);
```
Qui permet de récupérer n'importe quelle instance.