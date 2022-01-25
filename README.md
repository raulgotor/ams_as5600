<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Thanks again! Now go create something AMAZING! :D
***
***
***
*** To avoid retyping too much info. Do a search and replace for the following:
*** raulgotor, ams_as5600, twitter_handle, AMS AS5600 Driver, C driver for the AMS AS5600 magnetic rotary position sensor
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/raulgotor/ams_as5600">
    <!img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">AMS AS5600 Driver</h3>

  <p align="center">
    C driver for the AMS AS5600 magnetic rotary position sensor
    <br />
    <a href="https://github.com/raulgotor/ams_as5600"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/raulgotor/ams_as5600">View Demo</a>
    ·
    <a href="https://github.com/raulgotor/ams_as5600/issues">Report Bug</a>
    ·
    <a href="https://github.com/raulgotor/ams_as5600/issues">Request Feature</a>
  </p>

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT 
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

Here's a blank template to get started:
**To avoid retyping too much info. Do a search and replace with your text editor for the following:**
`raulgotor`, `ams_as5600`, `twitter_handle`, `AMS AS5600 Driver`, `C driver for the AMS AS5600 magnetic rotary position sensor`


### Built With

* []()
* []()
* []()

-->

<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Installation

1. Navigate to your project's source directory

2. Clone the repo
   ```sh
   git clone https://github.com/raulgotor/ams_as5600.git
   ```
3. Write a transfer function (see next section)


<!-- USAGE EXAMPLES -->
## Usage

### Transfer function

A transfer function is needed by the driver to know how to use the I2C
of your specific hardware. Above, a simple example of how this could be implemented
for STM32 MCUs using the STM32 HAL Libraries is shown:

```
I2C_HandleTypeDef * hi2c;

uint32_t const my_i2c_xfer(uint8_t const slave_address,
                           uint8_t const * const p_tx_buffer,
                           size_t const tx_buffer_size,
                           uint8_t * const p_rx_buffer,
                           size_t const rx_buffer_size)
{

        uint32_t const timeout = 100;
        HAL_StatusTypeDef result = HAL_OK;
        bool is_rx_operation = true;
        
        if ((NULL == p_tx_buffer) || (0 == tx_buffer_size)) {
                result = HAL_ERROR;

        } else if ((NULL == p_rx_buffer) || (0 == rx_buffer_size)) {
                is_rx_operation = false;
        }

        if (HAL_OK == result) {
                // TX operation
                result = HAL_I2C_Master_Transmit(hi2c,
                                                 slave_address,
                                                 p_tx_buffer,
                                                 (uint16_t)tx_buffer_size,
                                                 timeout);
        }
        
        if ((HAL_OK == result) && (is_rx_operation)) {
                // RX operation
                result = HAL_I2C_Master_Receive(hi2c,
                                                slave_address,
                                                p_rx_buffer,
                                                rx_buffer_size,
                                                timeout);
        }
        
        return result;
}
```

### Module initialization

After declaring the transfer function, initialize the module with the following call:

```
as5600_error_t result = as5600_init(my_i2c_xfer);
```

### Further documentation

_Please refer to the in code documentation and to the [AMS AS5600 Datasheet](https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf/649ee61c-8f9a-20df-9e10-43173a3eb323)_



<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/raulgotor/ams_as5600/issues) for a list of proposed features (and known issues).



<!-- CONTRIBUTING -->
## Contributing

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

Raúl Gotor

Project Link: [https://github.com/raulgotor/ams_as5600](https://github.com/raulgotor/ams_as5600)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Best README template](https://github.com/othneildrew/Best-README-Template)


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/raulgotor/ams_as5600.svg?style=for-the-badge
[contributors-url]: https://github.com/raulgotor/ams_as5600/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/raulgotor/ams_as5600.svg?style=for-the-badge
[forks-url]: https://github.com/raulgotor/ams_as5600/network/members
[stars-shield]: https://img.shields.io/github/stars/raulgotor/ams_as5600.svg?style=for-the-badge
[stars-url]: https://github.com/raulgotor/ams_as5600/stargazers
[issues-shield]: https://img.shields.io/github/issues/raulgotor/ams_as5600.svg?style=for-the-badge
[issues-url]: https://github.com/raulgotor/ams_as5600/issues
[license-shield]: https://img.shields.io/github/license/raulgotor/ams_as5600.svg?style=for-the-badge
[license-url]: https://github.com/raulgotor/ams_as5600/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/raulgotor