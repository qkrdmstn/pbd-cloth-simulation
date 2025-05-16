## 📑 목차
- [🧾 프로젝트 개요](#-프로젝트-개요)
- [📆 개발 인원 및 기간](#-개발-인원-및-기간)
- [🛠️ 주요 기능](#️-주요-기능)
- [💻 사용 기술](#-사용-기술)
- [📷 결과 영상](#-결과-영상)
- [🏆 프로젝트 성과](#-프로젝트-성과)
- [💡 프로젝트 회고](#-프로젝트-회고)
- [🔖 관련 블로그 글](#-관련-블로그-글)
- [📚 참고 자료](#-참고-자료)

<br>

## 🧾 프로젝트 개요
- 이 프로젝트는 **Position-Based Dynamics(PBD) 기법**을 활용한 실시간 **옷감(Cloth) 시뮬레이션**입니다.  
- OpenGL을 사용하여 직접 시뮬레이션을 구현했으며, **충돌 처리 및 자기 충돌(self-collision)** 등의 기능을 추가하여 천의 자연스러운 움직임을 표현했습니다.
- 본 프로젝트는 **게임 개발과 그래픽스 분야**에서 자주 활용되는 **핵심 기술들을 심층적으로 이해**하고 직접 구현하기 위해 진행되었습니다.

<br>

## 📆 개발 인원 및 기간
- 1인 개발
- 2023년 6월 ~ 2024년 1월 (약 8개월)

<br>

## 🛠️ 주요 기능
- 옷감 시뮬레이션
- 강체와의 충돌 처리
- 자가 충돌 (self-collision)

<br>

## 💻 사용 기술
### 개발 언어

<img src="https://img.shields.io/badge/c-A8B9CC?style=for-the-badge&logo=c&logoColor=white"> <img src="https://img.shields.io/badge/c++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white">

### 라이브러리
<img src="https://img.shields.io/badge/opengl-5586A4?style=for-the-badge&logo=opengl&logoColor=white">

### 시뮬레이션 기법
- Position-Based Dynamics (PBD)
- Self-Collision
- Collision Detection & Response
- Spatial Hashing
- Signed Distance Field (SDF)

<br>

## 📷 결과 영상
<p align="center">
  <img src="https://github.com/user-attachments/assets/d5d32641-b39b-4eab-a043-562b6b4733d0" width="45%">
  <img src="https://github.com/user-attachments/assets/1067f577-b406-459e-b1f0-2a61fac2f832" width="45%"><br>
  <img src="https://github.com/user-attachments/assets/d2097e2b-128f-4d64-b414-85a646177ce7" width="45%">
  <img src="https://github.com/user-attachments/assets/9dc210b0-02c6-498a-8291-258b876346b7" width="45%">
</p>

[🔗 결과 영상 링크 (Youtube)](https://www.youtube.com/playlist?list=PLL7N-Nw3U-P1VskT4llhvH_EJs00NhZ-c)

<br>

## 🏆 프로젝트 성과
본 프로젝트를 바탕으로 **2024 한국컴퓨터정보학회 동계 학술대회**에 **제1저자 및 발표자**로 참가하였습니다.

논문 제목: 옷감-고체 충돌에서 떨림 문제를 줄이기 위한 효율적인 SDF 기반 접근 방식

[🔗 논문 링크(DBpia)](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11711777)

<br>

## 💡 프로젝트 회고
- 이 프로젝트를 통해 게임 개발에 활용되는 물리 기반 시뮬레이션의 개념과 구현 방식을 실제로 체험할 수 있었습니다.
- 옷감 시뮬레이션은 단순한 시각 효과를 넘어, 캐릭터의 의상이나 환경의 상호작용 같은 게임 내 다양한 요소에 자연스러움을 부여하는 핵심 기술입니다.
  특히 **Position-Based Dynamics 기법**과 **충돌 처리 및 자가 충돌(self-collision)** 구현 과정을 통해, 현실적인 움직임을 만드는 데 필요한 물리 제약 조건들을 어떻게 모델링하고 해결해야 하는지를 깊이 있게 이해할 수 있었습니다.
- 이번 프로젝트를 통해 게임 클라이언트 개발자로서 **물리 기반 로직을 효과적으로 응용**하고, **실제 게임 콘텐츠의 몰입도를 높이는 방향**으로 기술을 적용할 수 있는 기반을 마련하게 되었습니다.

<br>

## 🔖 관련 블로그 글
- [🔗 자세한 구현 및 학습 과정 정리 (Tistory)](https://coding-l7.tistory.com/category/%EB%AC%BC%EB%A6%AC%20%EA%B8%B0%EB%B0%98%20%EC%8B%9C%EB%AE%AC%EB%A0%88%EC%9D%B4%EC%85%98/Cloth%20Simulation)

<br>
  
## 📚 참고 자료

### Position Based Dynamics
- [Position Based Dynamics - Matthias Müller, Bruno Heidelberger, Marcus Hennix, John Ratcliff](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf)

### Self-Collision
- [Cloth Self Collision with Predictive Contacts - Chris Lewin](https://media.contentapi.ea.com/content/dam/eacom/frostbite/files/gdc2018-chrislewin-clothselfcollisionwithpredictivecontacts.pdf)
- https://matthias-research.github.io/pages/tenMinutePhysics/index.html

### Signed Distance Field
- [Generating Signed Distance Fields From Triangle Meshes - J. Andreas Bærentzen and Henrik Aanæs](https://www2.imm.dtu.dk/pubdb/edoc/imm1289.pdf)

### Collision Response
- [Simulation of Clothing with Folds and Wrinkles - R. Bridson, S. Marino, R. Fedkiw](http://physbam.stanford.edu/~fedkiw/papers/stanford2003-06.pdf)
- [Robust High-Resolution Cloth Using Parallelism, History-Based Collisions and Accurate Friction - Andrew Selle et al.](http://physbam.stanford.edu/~fedkiw/papers/stanford2007-06.pdf)

### Collision Detection
- [Local Optimization for Robust Signed Distance Field Collision - Miles Macklin et al.](https://mmacklin.com/sdfcontact.pdf)
