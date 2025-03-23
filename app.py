import streamlit as st
import importlib


st.markdown(
    """
    <style>
    /* Estilo para o container principal */
    .main-container {
        background-color: #f0f2f6;
        padding: 2rem;
        border-radius: 10px;
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }
    /* Estilo para os botões */
    .stButton>button {
        background-color: #4CAF50;
        color: white;
        border: none;
        padding: 10px 24px;
        border-radius: 8px;
        font-size: 16px;
    }
    </style>
    """,
    unsafe_allow_html=True
)

st.markdown('<div class="main-container">', unsafe_allow_html=True)

st.title("Simulador de Missão do Drone")

versao = st.sidebar.radio("Selecione a versão do código:", 
                           ("Sky case v1", "Sky case v2", "Sky case v3"))

if st.button("Executar Código"):
    # Importa o módulo selecionado dinamicamente
    if versao == "Sky case v1":
        codigo = importlib.import_module("sky_case_v1")  
    elif versao == "Sky case v2":
        codigo = importlib.import_module("sky_case_v2")
    else:
        codigo = importlib.import_module("sky_case_v3")

    st.write(f"Executando {versao}...")
    if hasattr(codigo, "main"):
        resultado = codigo.main()  
        st.pyplot(resultado)
    else:
        st.error("O módulo selecionado não possui uma função main().")

st.markdown('</div>', unsafe_allow_html=True)
