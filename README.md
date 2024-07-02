<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
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
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <!--
  <a href="https://github.com/Ragarr/Kinect-Scanner">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>
  -->

<h3 align="center">Kinect 3D Scanner</h3>

  <p align="center">
    This uses Kinect v1 technology to scan objects and generate 3D models.
    It uses the pykinect library (adapted to python 3) together with the open3d library.
    <br />
    <a href="https://github.com/Ragarr/Kinect-Scanner"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/Ragarr/Kinect-Scanner">Go to repo</a>
    ·
    <a href="https://github.com/Ragarr/Kinect-Scanner/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/Ragarr/Kinect-Scanner/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
        <li><a href="#usage">Usage</a></li>
      </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

#REPLACE



<!-- GETTING STARTED -->
## Instalation

### Clone the repo / Download the zip file

```sh
git clone https://github.com/Ragarr/Kinect-Scanner.git
# gh repo clone Ragarr/Kinect-Scanner
cd Kinect-Scanner
```

### Install the Kinect SDK
Download and install the Kinect SDK from the official Microsoft page. You can find it [here](https://www.microsoft.com/en-us/download/details.aspx?id=40278)
 
### Crete enviroment and install dependences
Create a python 3.11 env, at the moment open3d is not compatible with python 3.12

`python -m venv .venv`

Activate the env (Windows)
 
``.venv\Scripts\activate``

Activate the env (Linux and MacOS)
    
``source .venv/bin/activate``


Install the dependences
    
``pip install -r requirements.txt``


## Usage

#TO DO


<!-- ROADMAP -->
## Roadmap

- [x] Capturing Kinect Depth Images
- [x] Capturing Kinect RGB + Depth Images
- [x] Generating 3D Point Clouds from RGBD Images
- [ ] Merging different point clouds into a single one
- [ ] Smoothing Point Clouds
- [ ] Generating 3D Meshes from Point Clouds
- [ ] Exporting 3D Models to different formats
- [ ] Accelerating the process using GPU
- [ ] Implementing a GUI

See the [open issues](https://github.com/Ragarr/Kinect-Scanner/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Your Name - [ragarr.com](https://www.ragarr.com/) - raulagarr@gmail.com

Project Link: [https://github.com/Ragarr/Kinect-Scanner](https://github.com/Ragarr/Kinect-Scanner)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Ragarr/Kinect-Scanner.svg?style=for-the-badge
[contributors-url]: https://github.com/Ragarr/Kinect-Scanner/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Ragarr/Kinect-Scanner.svg?style=for-the-badge
[forks-url]: https://github.com/Ragarr/Kinect-Scanner/network/members
[stars-shield]: https://img.shields.io/github/stars/Ragarr/Kinect-Scanner.svg?style=for-the-badge
[stars-url]: https://github.com/Ragarr/Kinect-Scanner/stargazers
[issues-shield]: https://img.shields.io/github/issues/Ragarr/Kinect-Scanner.svg?style=for-the-badge
[issues-url]: https://github.com/Ragarr/Kinect-Scanner/issues
[license-shield]: https://img.shields.io/github/license/Ragarr/Kinect-Scanner.svg?style=for-the-badge
[license-url]: https://github.com/Ragarr/Kinect-Scanner/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/raul-aguilar-arroyo-208462221/
[product-screenshot]: images/screenshot.png