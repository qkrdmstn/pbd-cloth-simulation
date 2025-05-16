# 📌 옷감 시뮬레이션 (Cloth Simulation)
Position-Based Dynamics 기반으로 구현한 실시간 옷감 시뮬레이션 프로젝트입니다.


# 🧾 프로젝트 개요
이 프로젝트는 Position-Based Dynamics(PBD) 기법을 활용한 실시간 옷감(Cloth) 시뮬레이션입니다.  
OpenGL을 사용하여 직접 시뮬레이션을 구현했으며, 충돌 처리 및 자기 충돌(self-collision) 등의 기능을 추가하여 천의 자연스러운 움직임을 표현했습니다.  

본 프로젝트는 게임 개발과 그래픽스 응용에서 자주 사용되는 핵심 기술들을 심도 있게 학습하고자 진행되었습니다.

# 🛠️ 주요 기능
- 옷감 시뮬레이션
- 강체와의 충돌 처리
- 자가 충돌 (self-collision)
    
# 💻 사용 기술
- 언어: C/C++
- 라이브러리: OpenGL
- Position Based Dynamics
- Self-Collision
- Singed Distance Field
- Collision Response
- Collision Detection
- K-D tree

# 📷 시연 영상
<p align="center">
  <img src="https://github.com/user-attachments/assets/d5d32641-b39b-4eab-a043-562b6b4733d0" width="45%">
  <img src="https://github.com/user-attachments/assets/1067f577-b406-459e-b1f0-2a61fac2f832" width="45%"><br>
  <img src="https://github.com/user-attachments/assets/d2097e2b-128f-4d64-b414-85a646177ce7" width="45%">
  <img src="https://github.com/user-attachments/assets/9dc210b0-02c6-498a-8291-258b876346b7" width="45%">
</p>


# 📚 참고 자료
### Position Based Dynamics

- [Position Based Dynamics - Matthias Müller, Bruno Heidelberger, Marcus Hennix, John Ratcliff](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf)

### Self-Collision

- [Cloth Self Collision with Predictive Contacts - Chirs Lewin](https://media.contentapi.ea.com/content/dam/eacom/frostbite/files/gdc2018-chrislewin-clothselfcollisionwithpredictivecontacts.pdf)
- https://matthias-research.github.io/pages/tenMinutePhysics/index.html

### Singed Distance Field

- [Generating Signed Distance Fields From Triangle Meshes - J. Andreas Bærentzen and Henrik Aanæs](https://www2.imm.dtu.dk/pubdb/edoc/imm1289.pdf)

### Collision Response

- [Simulation of Clothing with Folds and Wrinkles - R. Bridson, S. Marino, R. Fedkiw](http://physbam.stanford.edu/~fedkiw/papers/stanford2003-06.pdf)
- [Robust High-Resolution Cloth Using Parallelism, History-Based Collisions and Accurate Friction - Andrew Selle, Jonathan Su, Geoffrey Irving, Ronald Fedkiw](http://physbam.stanford.edu/~fedkiw/papers/stanford2007-06.pdf)

### Collision Detection

- [Local Optimization for Robust Signed Distance Field Collision (MILESMACKLIN, KENNYERLEBEN, MATTHIASMÜLLER, NUTTAPONGCHENTANEZ, STEFANJESCHKE, ZACHCORSE)](https://mmacklin.com/sdfcontact.pdf)
